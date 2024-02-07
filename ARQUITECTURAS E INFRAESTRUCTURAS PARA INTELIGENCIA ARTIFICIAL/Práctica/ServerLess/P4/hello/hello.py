import http.server
import socketserver
# Define the request handler
class HelloHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        self.wfile.write(b'Hello, World!')
            # Set up the server
            
            
with socketserver.TCPServer(('', 80), HelloHandler) as httpd:
    print('Server started on port 80...')
    httpd.serve_forever()