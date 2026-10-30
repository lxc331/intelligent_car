"""Small OpenAI-compatible HTTP adapter; keys remain in request memory only.

This module returns text. It never interprets model output as Python, shell,
ROS messages or motion commands. The separate action validator owns that step.
"""
from dataclasses import dataclass, field
import base64
import ipaddress
import json
from pathlib import Path
import uuid
from urllib import error, parse, request


MAX_RESPONSE_BYTES = 1024 * 1024
MAX_AUDIO_BYTES = 5 * 1024 * 1024
PREFERENCE_KEYS = frozenset(("provider", "base_url", "model", "asr_provider",
                            "asr_base_url", "asr_model", "asr_mode"))


class AIError(RuntimeError):
    """User-facing errors contain no response body, key or transport detail."""


class AICancelled(AIError):
    pass


def _base_url(value):
    if not isinstance(value, str) or not value.strip() or len(value) > 2048:
        raise ValueError("请填写有效的 API Base URL")
    if any(ord(char) < 33 or ord(char) == 127 for char in value):
        raise ValueError("API 地址不能包含空格或控制字符")
    try:
        parts = parse.urlsplit(value)
        host, port = parts.hostname, parts.port
        if not host or parts.username is not None or parts.password is not None or "?" in value or "#" in value:
            raise ValueError
        local = host.lower() == "localhost"
        try:
            local = local or ipaddress.ip_address(host).is_loopback
        except ValueError:
            pass
        if parts.scheme != "https" and not (parts.scheme == "http" and local):
            raise ValueError
        if port is not None and not 0 < port <= 65535:
            raise ValueError
        value.encode("ascii")
    except (ValueError, UnicodeError):
        raise ValueError("API 地址须为 HTTPS，且不能包含用户名、密码、查询参数或片段；本机接口可用 HTTP") from None
    if parts.path.rstrip("/").endswith(("/chat/completions", "/audio/transcriptions")):
        raise ValueError("Base URL 请填写到 /v1 等基础路径，不要包含 /chat/completions 或 /audio/transcriptions")
    return value.rstrip("/")


def _text(value, label, limit):
    if not isinstance(value, str) or not value.strip() or len(value) > limit:
        raise ValueError("%s为空或过长" % label)
    if any(ord(char) < 32 or ord(char) == 127 for char in value):
        raise ValueError("%s包含控制字符" % label)
    return value.strip()


@dataclass(frozen=True)
class AISettings:
    base_url: str
    model: str
    api_key: str = field(repr=False)

    def __post_init__(self):
        object.__setattr__(self, "base_url", _base_url(self.base_url))
        object.__setattr__(self, "model", _text(self.model, "模型名称", 256))
        object.__setattr__(self, "api_key", _text(self.api_key, "API Key", 8192))
        if any(char.isspace() for char in self.api_key):
            raise ValueError("API Key 不能包含空白字符")


class _NoRedirect(request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        # Never forward Authorization to a redirected or substituted endpoint.
        raise AIError("API 返回重定向，请直接填写服务商提供的最终 Base URL")


def _cancelled(event):
    if event is not None and event.is_set():
        raise AICancelled("请求已取消")


def _http_message(code):
    if code in (401, 403):
        return "API 认证失败，请检查 Key、所属服务商和权限"
    if code == 404:
        return "API 地址或模型不存在，请检查 Base URL 和模型名称"
    if code == 429:
        return "API 请求受限，请检查额度或稍后重试"
    if code in (400, 422):
        return "API 不接受此请求，请检查模型和接口类型是否支持"
    return "API 请求失败（HTTP %s），请稍后重试或检查服务商状态" % code


class AIClient:
    def __init__(self, settings, timeout=25):
        if not isinstance(settings, AISettings):
            raise TypeError("需要 AISettings 配置")
        self.settings = settings
        self.timeout = min(30., max(1., float(timeout)))
        self._opener = request.build_opener(_NoRedirect())

    def _request(self, suffix, data, content_type, cancel_event=None):
        _cancelled(cancel_event)
        req = request.Request(self.settings.base_url + suffix, data=data, method="POST", headers={
            "Authorization": "Bearer " + self.settings.api_key,
            "Content-Type": content_type,
            "Accept": "application/json",
            "User-Agent": "ClawPi-Desktop-AI/1",
        })
        try:
            with self._opener.open(req, timeout=self.timeout) as response:
                code = getattr(response, "status", 200)
                if code >= 400:
                    raise AIError(_http_message(code))
                raw = response.read(MAX_RESPONSE_BYTES + 1)
        except error.HTTPError as exc:
            raise AIError(_http_message(exc.code)) from None
        except (error.URLError, OSError, TimeoutError, ValueError):
            raise AIError("API 连接失败或超时，请检查网络和接口地址") from None
        _cancelled(cancel_event)
        if len(raw) > MAX_RESPONSE_BYTES:
            raise AIError("API 响应过大，已停止处理")
        try:
            value = json.loads(raw.decode("utf-8"), parse_constant=lambda value: (_ for _ in ()).throw(ValueError()))
        except (ValueError, UnicodeError, RecursionError):
            raise AIError("API 没有返回有效 JSON") from None
        if not isinstance(value, dict) or "error" in value:
            raise AIError("API 返回错误，请检查模型、额度和接口配置")
        return value

    def _json_request(self, payload, cancel_event=None):
        raw = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode("utf-8")
        return self._request("/chat/completions", raw, "application/json", cancel_event)

    def _content(self, response):
        try:
            choice = response["choices"][0]
            if choice.get("finish_reason") == "length":
                raise AIError("模型回答被截断，请缩短指令后重新生成")
            text = choice["message"]["content"]
            if isinstance(text, list):
                text = "".join(part["text"] for part in text
                               if isinstance(part, dict) and isinstance(part.get("text"), str))
            if not isinstance(text, str) or not text.strip() or len(text) > 32768:
                raise ValueError
        except (KeyError, IndexError, TypeError, ValueError, AttributeError):
            raise AIError("模型没有返回有效文字，请检查模型是否支持对话接口") from None
        if len(self.settings.api_key) >= 8 and self.settings.api_key in text:
            raise AIError("服务返回内容包含敏感凭据，已丢弃")
        return text.strip()

    def _chat(self, messages, max_tokens, cancel_event=None):
        payload = dict(model=self.settings.model, messages=messages, stream=False, max_tokens=max_tokens)
        host = parse.urlsplit(self.settings.base_url).hostname.lower()
        if (host.endswith(".aliyuncs.com") or host == "dashscope.aliyuncs.com") and self.settings.model.startswith("qwen"):
            # Mixed-thinking Qwen models support this explicit non-streaming mode.
            payload["enable_thinking"] = False
        elif host == "api.deepseek.com":
            payload["thinking"] = {"type": "disabled"}
        return self._content(self._json_request(payload, cancel_event))

    def check_connection(self):
        self._chat([{"role": "user", "content": "只回复 OK"}], 16)
        return "连接成功，模型已响应"

    def plan(self, text, system_prompt, cancel_event=None):
        if not isinstance(text, str) or not text.strip() or len(text) > 4000:
            raise ValueError("请输入不超过 4000 字的指令")
        if not isinstance(system_prompt, str) or not system_prompt.strip() or len(system_prompt) > 16000:
            raise ValueError("动作提示词为空或过长")
        return self._chat([{"role": "system", "content": system_prompt},
                           {"role": "user", "content": text.strip()}], 2048, cancel_event)

    def transcribe(self, wav_bytes, model, *, mode="openai", cancel_event=None):
        if parse.urlsplit(self.settings.base_url).hostname.lower() == "api.deepseek.com":
            raise ValueError("DeepSeek 文本接口不用于语音识别，请单独配置 ASR 服务或使用 Windows 听写")
        model = _text(model, "语音识别模型名称", 256)
        if (not isinstance(wav_bytes, bytes) or not 44 <= len(wav_bytes) <= MAX_AUDIO_BYTES
                or wav_bytes[:4] != b"RIFF" or wav_bytes[8:12] != b"WAVE"):
            raise ValueError("请选择不超过 5 MB 的有效 WAV 录音")
        if mode == "dashscope":
            # Qwen3-ASR-Flash OpenAI-compatible documented audio message format.
            audio = "data:audio/wav;base64," + base64.b64encode(wav_bytes).decode("ascii")
            payload = dict(model=model, stream=False, messages=[{"role": "user", "content": [
                {"type": "input_audio", "input_audio": {"data": audio}}]}], asr_options={"enable_itn": False})
            text = self._content(self._json_request(payload, cancel_event))
            if len(text) > 4000:
                raise AIError("语音识别文字过长，请缩短录音后重试")
            return text
        if mode != "openai":
            raise ValueError("不支持此语音接口类型")
        boundary = "clawpi" + uuid.uuid4().hex
        body = ("--%s\r\nContent-Disposition: form-data; name=\"model\"\r\n\r\n%s\r\n"
                "--%s\r\nContent-Disposition: form-data; name=\"response_format\"\r\n\r\njson\r\n"
                "--%s\r\nContent-Disposition: form-data; name=\"file\"; filename=\"command.wav\"\r\n"
                "Content-Type: audio/wav\r\n\r\n" % (boundary, model, boundary, boundary)).encode("utf-8")
        body += wav_bytes + ("\r\n--%s--\r\n" % boundary).encode("ascii")
        response = self._request("/audio/transcriptions", body, "multipart/form-data; boundary=" + boundary, cancel_event)
        text = response.get("text")
        if not isinstance(text, str) or not text.strip() or len(text) > 4000:
            raise AIError("语音接口没有返回有效文字")
        if len(self.settings.api_key) >= 8 and self.settings.api_key in text:
            raise AIError("服务返回内容包含敏感凭据，已丢弃")
        return text.strip()


def _preferences(values):
    result = {}
    for key in PREFERENCE_KEYS:
        value = values.get(key)
        if isinstance(value, str) and len(value) <= 2048:
            if key.endswith("base_url") and value:
                value = _base_url(value)
            result[key] = value.strip()
    return result


def load_preferences(path):
    try:
        path = Path(path)
        if path.stat().st_size > 65536:
            return {}
        data = json.loads(path.read_text(encoding="utf-8"))
        return _preferences(data) if isinstance(data, dict) else {}
    except (OSError, ValueError, UnicodeError):
        return {}


def save_preferences(path, values):
    """Atomic allowlisted settings write; api_key/token/password never persist."""
    data = _preferences(values)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)
