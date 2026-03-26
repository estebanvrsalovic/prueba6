#!/usr/bin/env python3
from http.server import BaseHTTPRequestHandler, HTTPServer
import threading

HOST = '127.0.0.1'
PORT = 8001
SAVED = '/tmp/test_device_schedule_received.csv'
SAVED_JSON = '/tmp/test_schedules.json'

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith('/device_schedule'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/csv')
            self.end_headers()
            # sample CSV: epoch_ms,relay,duration,repeat8
            body = '1700000000000,1,60,0\n1700000005000,2,30,0\n'
            self.wfile.write(body.encode())
        elif self.path.startswith('/functions/v1/schedules') or self.path.startswith('/schedules'):
            # Return sample JSON array similar to Supabase REST API
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            body = '[{"id":1,"epoch_ms":1700000000000,"action":"on","zone":1,"meta":{}},{"id":2,"epoch_ms":1700000005000,"action":"on","zone":2,"meta":{}}]'
            self.wfile.write(body.encode())
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path.startswith('/device_schedule'):
            length = int(self.headers.get('content-length', '0'))
            data = self.rfile.read(length) if length > 0 else b''
            # save received CSV to file for inspection
            try:
                with open(SAVED, 'wb') as f:
                    f.write(data)
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(b'{"ok":true,"saved":"%s"}' % SAVED.encode())
            except Exception as e:
                self.send_response(500)
                self.end_headers()
                self.wfile.write(str(e).encode())
        elif self.path.startswith('/functions/v1/schedules') or self.path.startswith('/schedules'):
            # accept JSON body and save to file
            length = int(self.headers.get('content-length', '0'))
            data = self.rfile.read(length) if length > 0 else b''
            try:
                with open(SAVED_JSON, 'wb') as f:
                    f.write(data)
                self.send_response(201)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(b'{"ok":true}')
            except Exception as e:
                self.send_response(500)
                self.end_headers()
                self.wfile.write(str(e).encode())
        else:
            self.send_response(404)
            self.end_headers()

def run():
    httpd = HTTPServer((HOST, PORT), Handler)
    print('Test device_schedule server running on http://%s:%s' % (HOST, PORT))
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    httpd.server_close()

if __name__ == '__main__':
    run()
