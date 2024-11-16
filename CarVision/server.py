from http.server import BaseHTTPRequestHandler, HTTPServer
import subprocess, configparser, urllib

def data_process(data: str):
    temp = data.split(' ')
    while '' in temp:
        temp.remove('')
    temp = ''.join([str(i) for i in temp]).split(',')
    while '' in temp:
        temp.remove('')
    return [int(i) for i in temp]

class WebHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        if self.path == '/hsv_editor':
            self.handle_index()
        else:
            self.handle_err()

    def handle_index(self):
        text_code = subprocess.getoutput('systemctl status car_vision.service')
        body_style = "<style>body{display:flex;justify-content:center;align-items:center;height:100vh;margin:0;padding:0}*{padding-right:10px}</style>"
        hsv_code = f"\
Red HSV 1:\n{[int(i) for i in config.get('using', 'red_hsv_1')[1:-1].split(',')]}\n\n\
Red HSV 2:\n{[int(i) for i in config.get('using', 'red_hsv_2')[1:-1].split(',')]}\n\n\
Green HSV:\n{[int(i) for i in config.get('using', 'green_hsv')[1:-1].split(',')]}\n\n\
Blue HSV:\n{[int(i) for i in config.get('using', 'blue_hsv')[1:-1].split(',')]}\n\n\
Circle HSV:\n{[int(i) for i in config.get('using', 'circle_hsv')[1:-1].split(',')]}\n\n"

        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()
        self.wfile.write(f"<html>{body_style}<body>".encode("utf-8"))
        self.wfile.write(b"<form method='POST' action='/edit'>")
        self.wfile.write(b"<h1>HSV Setter</h1>")
        self.wfile.write(b"<input type='text' name='r1' placeholder='Red HSV 1' autocomplete='off'><br><br>")
        self.wfile.write(b"<input type='text' name='r2' placeholder='Red HSV 2' autocomplete='off'><br><br>")
        self.wfile.write(b"<input type='text' name='g' placeholder='Green HSV' autocomplete='off'><br><br>")
        self.wfile.write(b"<input type='text' name='b' placeholder='Blue HSV' autocomplete='off'><br><br>")
        self.wfile.write(b"<input type='text' name='c' placeholder='Circle HSV' autocomplete='off'><br><br>")
        self.wfile.write(b"<input type='submit' value='Submit'><br><br><br>")
        self.wfile.write(b"<textarea rows=12 cols=32>")
        self.wfile.write(f"{hsv_code}".encode("utf-8"))
        self.wfile.write(b"</textarea><br><br></form>")
        self.wfile.write(b"<button onclick='location.reload(true)'>Reload<br>")
        self.wfile.write(b"<textarea rows=30 cols=100 readonly>")
        self.wfile.write(f"{text_code}".encode("utf-8"))
        self.wfile.write(b"</textarea></button></body></html>")

    def handle_err(self):
        self.send_response_only(404)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

    def do_POST(self):
        if self.path == '/edit':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            form_data = post_data.decode('utf-8').split('&')

            r1 = urllib.parse.unquote(form_data[0].split('=')[1])
            r2 = urllib.parse.unquote(form_data[1].split('=')[1])
            g = urllib.parse.unquote(form_data[2].split('=')[1])
            b = urllib.parse.unquote(form_data[3].split('=')[1])
            c = urllib.parse.unquote(form_data[4].split('=')[1])

            changed = False

            data = data_process(r1)
            if len(data) == 6:
                config.set('using', 'red_hsv_1', str(data))
                changed = True

            data = data_process(r2)
            if len(data) == 6:
                config.set('using', 'red_hsv_2', str(data))
                changed = True

            data = data_process(g)
            if len(data) == 6:
                config.set('using', 'green_hsv', str(data))
                changed = True

            data = data_process(b)
            if len(data) == 6:
                config.set('using', 'blue_hsv', str(data))
                changed = True

            data = data_process(c)
            if len(data) == 6:
                config.set('using', 'circle_hsv', str(data))
                changed = True

            if changed:
                with open('./config.ini', 'w') as configfile:
                    config.write(configfile)

            self.send_response(303)
            self.send_header('Location', '/hsv_editor')
            self.end_headers()
        else:
            self.handle_err()

def run(server_class=HTTPServer, handler_class=WebHandler, port=8000):
    server_address = ('0.0.0.0', port)
    httpd = server_class(server_address, handler_class)
    print(f'Starting httpd server on port {port}...')
    httpd.serve_forever()

if __name__ == '__main__':
    config = configparser.ConfigParser()
    config.read('./config.ini')

    run(port = 8888)
