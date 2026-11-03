"""Read-only probe: what does the vehicle expose as its identity?"""
import paramiko

client = paramiko.SSHClient()
client.load_system_host_keys()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect("192.168.150.2", username="pi", password="raspberrypi", timeout=6, auth_timeout=8)
for command in ('tr -d "\\0\\n" < /proc/device-tree/serial-number; echo',
                'hostname',
                'ls /proc/device-tree/model && tr -d "\\0" < /proc/device-tree/model; echo'):
    _, out, err = client.exec_command(command, timeout=8)
    print(command, '=>', repr(out.read().decode('utf-8', 'replace').strip()),
          '| err:', repr(err.read().decode('utf-8', 'replace').strip()[:120]))
client.close()
