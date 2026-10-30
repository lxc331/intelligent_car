"""AI transport, cancellation and credential boundaries; no external requests."""
import base64
import io
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch
from urllib.error import HTTPError, URLError
from urllib.request import Request

from carapp.core import ai_client as ai


KEY = "test-key-never-real"


def completion(text="OK", finish_reason="stop"):
    return {"choices": [{"finish_reason": finish_reason, "message": {"content": text}}]}


class Reply:
    status = 200

    def __init__(self, value, hook=None):
        self.raw = value if isinstance(value, bytes) else json.dumps(value).encode("utf-8")
        self.hook = hook
        self.closed = False
        self.read_limit = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.closed = True

    def read(self, limit):
        self.read_limit = limit
        if self.hook:
            self.hook()
        return self.raw[:limit]


class AISettingsTests(unittest.TestCase):
    def test_key_is_hidden_and_urls_accept_custom_https_or_local_http(self):
        for url in ("https://api.deepseek.com", "https://host.invalid/v1/", "http://127.0.0.1:1234/v1",
                    "http://localhost:8080/v1", "http://[::1]:8000/v1"):
            with self.subTest(url=url):
                value = ai.AISettings(url, "dummy-model", KEY)
                self.assertNotIn(KEY, repr(value))
                self.assertFalse(value.base_url.endswith("/"))

    def test_rejects_nonlocal_cleartext_credentials_query_fragment_and_control(self):
        for url in ("http://service.invalid/v1", "ftp://service.invalid/v1", "https://u:p@service.invalid/v1",
                    "https://service.invalid/v1?key=secret", "https://service.invalid/v1#key",
                    "https://service.invalid/v1\r\nHeader: data", "https://service.invalid:99999/v1",
                    "https:///v1", "https://service.invalid/v1/chat/completions"):
            with self.subTest(url=url), self.assertRaises(ValueError):
                ai.AISettings(url, "dummy", KEY)
        for url in ("https://service.invalid/v1?", "https://service.invalid/v1#", "https://service.invalid/\x7f"):
            with self.subTest(url=url), self.assertRaises(ValueError):
                ai.AISettings(url, "dummy", KEY)

    def test_rejects_blank_models_and_header_injection_keys(self):
        for model, key in (("", KEY), ("dummy", ""), ("dummy", "bad key"),
                           ("dummy", "key\r\nHeader:x"), ("bad\nmodel", KEY)):
            with self.subTest(model=model), self.assertRaises(ValueError):
                ai.AISettings("https://service.invalid/v1", model, key)


class AIClientTests(unittest.TestCase):
    def setUp(self):
        self.client = ai.AIClient(ai.AISettings("https://service.invalid/v1", "dummy", KEY))
        self.opener = Mock()
        self.client._opener = self.opener

    def answer(self, value, hook=None):
        response = Reply(value, hook)
        self.opener.open.return_value = response
        return response

    def request_payload(self):
        return json.loads(self.opener.open.call_args.args[0].data)

    def test_connection_test_is_an_api_ping_without_an_action_prompt(self):
        response = self.answer(completion())
        self.assertIn("连接成功", self.client.check_connection())
        req = self.opener.open.call_args.args[0]
        self.assertEqual(req.full_url, "https://service.invalid/v1/chat/completions")
        self.assertEqual(req.get_header("Authorization"), "Bearer " + KEY)
        self.assertNotIn(KEY.encode(), req.data)
        body = self.request_payload()
        self.assertEqual(body["messages"], [{"role": "user", "content": "只回复 OK"}])
        self.assertFalse(body["stream"])
        self.assertTrue(response.closed)
        self.assertEqual(response.read_limit, ai.MAX_RESPONSE_BYTES + 1)

    def test_plan_uses_separate_system_user_messages_and_never_executes_reply(self):
        malicious = "__import__('os').system('should never run')"
        self.answer(completion(malicious))
        result = self.client.plan("前进一点", "Only JSON")
        self.assertEqual(result, malicious)
        body = self.request_payload()
        self.assertEqual(body["model"], "dummy")
        self.assertEqual(body["messages"], [{"role": "system", "content": "Only JSON"},
                                           {"role": "user", "content": "前进一点"}])

    def test_provider_specific_non_thinking_fields_stay_on_their_host(self):
        cases = (("https://api.deepseek.com", "deepseek-flash", "thinking"),
                 ("https://dashscope.aliyuncs.com/compatible-mode/v1", "qwen-plus", "enable_thinking"),
                 ("https://workspace.cn-beijing.maas.aliyuncs.com/compatible-mode/v1", "qwen-plus", "enable_thinking"),
                 ("https://custom.invalid/v1", "qwen-local", None))
        for url, model, field in cases:
            with self.subTest(url=url):
                client = ai.AIClient(ai.AISettings(url, model, KEY))
                client._opener = Mock()
                client._opener.open.return_value = Reply(completion())
                client.plan("hello", "test prompt")
                body = json.loads(client._opener.open.call_args.args[0].data)
                self.assertEqual("thinking" in body, field == "thinking")
                self.assertEqual("enable_thinking" in body, field == "enable_thinking")

    def test_cancelled_before_request_does_no_io_and_late_reply_is_dropped(self):
        cancel = threading.Event()
        cancel.set()
        with self.assertRaises(ai.AICancelled):
            self.client.plan("前进", "prompt", cancel)
        self.opener.open.assert_not_called()
        cancel.clear()
        self.answer(completion("late action"), hook=cancel.set)
        with self.assertRaises(ai.AICancelled):
            self.client.plan("前进", "prompt", cancel)

    def test_http_errors_never_echo_key_or_untrusted_response_body(self):
        for status in (400, 401, 403, 404, 422, 429, 500):
            with self.subTest(status=status):
                self.opener.open.side_effect = HTTPError("https://service.invalid", status,
                    "Bearer " + KEY, {}, io.BytesIO(("error " + KEY).encode()))
                with self.assertRaises(ai.AIError) as caught:
                    self.client.plan("move", "prompt")
                self.assertNotIn(KEY, str(caught.exception))
                self.assertNotIn("Bearer", str(caught.exception))

    def test_transport_errors_are_safe_fixed_text(self):
        for failure in (URLError(KEY), TimeoutError(KEY), OSError(KEY)):
            with self.subTest(error=type(failure).__name__):
                self.opener.open.side_effect = failure
                with self.assertRaises(ai.AIError) as caught:
                    self.client.check_connection()
                self.assertNotIn(KEY, str(caught.exception))

    def test_json_shape_size_and_content_errors_fail_closed(self):
        for value in (b"not json", b'{"n":NaN}', [], {"error": {"message": KEY}},
                      {"choices": []}, {"choices": [None]}, {"choices": ["not an object"]}, completion(None), completion(""),
                      completion("x" * 33000), completion(KEY), completion("cut", "length")):
            with self.subTest(kind=type(value).__name__):
                self.answer(value)
                with self.assertRaises(ai.AIError) as caught:
                    self.client.plan("move", "prompt")
                self.assertNotIn(KEY, str(caught.exception))
        self.answer(b"x" * (ai.MAX_RESPONSE_BYTES + 10))
        with self.assertRaisesRegex(ai.AIError, "响应过大"):
            self.client.check_connection()

    def test_redirection_is_never_followed_with_authorization(self):
        handler = ai._NoRedirect()
        req = Request("https://service.invalid/v1/chat/completions",
                      headers={"Authorization": "Bearer " + KEY})
        for target in ("https://other.invalid/steal", "https://service.invalid/new"):
            with self.subTest(target=target), self.assertRaises(ai.AIError):
                handler.redirect_request(req, None, 307, "redirect", {}, target)

    def test_text_content_blocks_are_joined_and_missing_text_is_rejected(self):
        self.answer(completion([{"type": "text", "text": "hello"}, {"type": "text", "text": " world"}]))
        self.assertEqual(self.client.plan("hello", "prompt"), "hello world")
        self.answer(completion([{"type": "tool_call", "name": "execute"}]))
        with self.assertRaises(ai.AIError):
            self.client.plan("hello", "prompt")

    def test_empty_or_oversized_inputs_are_rejected_before_network(self):
        for text, prompt in (("", "p"), ("x" * 4001, "p"), ("x", ""), ("x", "p" * 16001)):
            with self.assertRaises(ValueError):
                self.client.plan(text, prompt)
        self.opener.open.assert_not_called()

    def test_dashscope_asr_uses_documented_audio_data_url_and_separate_model(self):
        wav = b"RIFF" + b"\x00" * 4 + b"WAVE" + b"\x00" * 40
        self.answer(completion("前进一秒"))
        self.assertEqual(self.client.transcribe(wav, "qwen3-asr-flash", mode="dashscope"), "前进一秒")
        body = self.request_payload()
        self.assertEqual(body["model"], "qwen3-asr-flash")
        self.assertEqual(body["messages"][0]["content"], [{"type": "input_audio", "input_audio": {
            "data": "data:audio/wav;base64," + base64.b64encode(wav).decode("ascii")}}])
        self.assertNotIn("enable_thinking", body)

    def test_multipart_asr_keeps_recording_in_memory_and_does_not_use_chat_model(self):
        wav = b"RIFF" + b"\x00" * 4 + b"WAVE" + b"\x00" * 40
        self.answer({"text": "向左转"})
        self.assertEqual(self.client.transcribe(wav, "test-asr"), "向左转")
        req = self.opener.open.call_args.args[0]
        self.assertEqual(req.full_url, "https://service.invalid/v1/audio/transcriptions")
        self.assertIn("multipart/form-data; boundary=", req.get_header("Content-type"))
        self.assertIn(b"test-asr", req.data)
        self.assertIn(wav, req.data)
        self.assertNotIn(KEY.encode(), req.data)

    def test_invalid_audio_and_asr_response_raise_safely(self):
        wav = b"RIFF" + b"\x00" * 4 + b"WAVE" + b"\x00" * 40
        for data, mode in ((b"invalid", "openai"), (wav, "unknown"),
                           (b"x" * (ai.MAX_AUDIO_BYTES + 1), "openai")):
            with self.assertRaises(ValueError):
                self.client.transcribe(data, "asr", mode=mode)
        self.opener.open.assert_not_called()
        for value in ({}, {"text": KEY}, {"text": []}):
            self.answer(value)
            with self.assertRaises(ai.AIError) as caught:
                self.client.transcribe(wav, "asr")
            self.assertNotIn(KEY, str(caught.exception))
        self.answer(b"[" * 2000 + b"0" + b"]" * 2000)
        with self.assertRaises(ai.AIError):
            self.client.check_connection()

    def test_deepseek_text_endpoint_never_receives_an_audio_upload(self):
        client = ai.AIClient(ai.AISettings("https://api.deepseek.com", "deepseek-flash", KEY))
        client._opener = Mock()
        wav = b"RIFF" + b"\x00" * 4 + b"WAVE" + b"\x00" * 40
        with self.assertRaisesRegex(ValueError, "单独配置 ASR"):
            client.transcribe(wav, "dummy-asr")
        client._opener.open.assert_not_called()


class AIPreferencesTests(unittest.TestCase):
    def test_only_nonsecret_allowlisted_fields_persist(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"ai_settings.json"
            values = {"provider": "DeepSeek", "base_url": "https://api.deepseek.com/", "model": "dummy",
                      "asr_base_url": "https://asr.invalid/v1", "asr_model": "asr", "asr_mode": "dashscope",
                      "api_key": KEY, "asr_api_key": KEY, "password": KEY, "nested": {"key": KEY}}
            ai.save_preferences(path, values)
            self.assertNotIn(KEY, path.read_text(encoding="utf-8"))
            loaded = ai.load_preferences(path)
            self.assertEqual(loaded["base_url"], "https://api.deepseek.com")
            self.assertEqual(loaded["model"], "dummy")
            self.assertNotIn("api_key", loaded)
            self.assertEqual(list(Path(folder).glob("*.tmp")), [])

    def test_bad_config_does_not_load_or_persist_secrets_in_urls(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"ai_settings.json"
            self.assertEqual(ai.load_preferences(path), {})
            path.write_text("not json", encoding="utf-8")
            self.assertEqual(ai.load_preferences(path), {})
            path.write_text(json.dumps({"base_url": "https://u:secret@bad.invalid/v1"}), encoding="utf-8")
            self.assertEqual(ai.load_preferences(path), {})
            with self.assertRaises(ValueError):
                ai.save_preferences(path, {"base_url": "https://bad.invalid/v1?api_key=" + KEY})
            self.assertNotIn(KEY, path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
