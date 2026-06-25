# coding: utf-8
import http.server
import urllib.parse
import json
import webbrowser
import threading

class CalibrationHTTPRequestHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        return

    def do_GET(self):
        state = self.server.state
        parsed_url = urllib.parse.urlparse(self.path)
        path = parsed_url.path
        if path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(state["HTML_CONTENT"].encode("utf-8"))
        elif path == "/get_offsets":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            data = {
                "offsets": state["offsets"].tolist(),
                "inversions": state["inversions"].tolist(),
                "control_pos": state["control_pos"].tolist(),
                "calibration_pos": state["calibration_pos"],
                "pins": state["RoboDog"].pins.tolist(),
                "dither_enabled": state.get("dither_enabled", False)
            }
            self.wfile.write(json.dumps(data).encode("utf-8"))
        elif path == "/get_status":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            adc_grid, nano1_d3, nano2_d3 = state["nano_controller"].get_adc_values()
            conn_status = state["nano_controller"].get_connection_status()
            data = {
                "offsets": state["offsets"].tolist(),
                "inversions": state["inversions"].tolist(),
                "control_pos": state["control_pos"].tolist(),
                "calibration_pos": state["calibration_pos"],
                "pins": state["RoboDog"].pins.tolist(),
                "nano_status": conn_status,
                "nano1_d3_adc": nano1_d3,
                "nano2_d3_adc": nano2_d3,
                "adc_grid": adc_grid,
                "dither_enabled": state.get("dither_enabled", False)
            }
            self.wfile.write(json.dumps(data).encode("utf-8"))
        elif path == "/get_soft_motion":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"use_soft_motion": state["use_soft_motion"]}).encode("utf-8"))
        else:
            self.send_error(404, "Not Found")

    def do_POST(self):
        state = self.server.state
        parsed_url = urllib.parse.urlparse(self.path)
        path = parsed_url.path
        
        if path == "/wiggle":
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length)
            params = json.loads(body.decode('utf-8'))
            nano_name = params.get("nano_name")
            state["nano_controller"].wiggle_d3(nano_name)
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"status": "ok"}).encode("utf-8"))
        elif path == "/set_mapping":
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length)
            params = json.loads(body.decode('utf-8'))
            mapping = params.get("mapping")
            if mapping:
                state["nano_controller"].mapping = {k: int(v) for k, v in mapping.items()}
                state["nano_controller"].save_config()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"status": "ok"}).encode("utf-8"))
        elif path == "/set_stance":
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length)
            params = json.loads(body.decode('utf-8'))
            stance_name = params.get("stance")
            
            status, res = state["set_stance"](stance_name)
            if status == "ok":
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "ok", "calibration_pos": res}).encode("utf-8"))
            else:
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "error", "message": res}).encode("utf-8"))
        elif path == "/set_soft_motion":
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length)
            params = json.loads(body.decode('utf-8'))
            state["use_soft_motion"] = bool(params.get("use_soft_motion", False))
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"status": "ok", "use_soft_motion": state["use_soft_motion"]}).encode("utf-8"))
        elif path == "/set_dither":
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length)
            params = json.loads(body.decode('utf-8'))
            state["dither_enabled"] = bool(params.get("dither_enabled", False))
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"status": "ok", "dither_enabled": state["dither_enabled"]}).encode("utf-8"))
        elif path == "/update_joint":
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length)
            params = json.loads(body.decode('utf-8'))
            
            leg_idx = int(params["leg_idx"])
            joint_idx = int(params["joint_idx"])
            offset = int(params["offset"]) if "offset" in params else None
            control = int(params["control"]) if "control" in params else None
            inversion = bool(params["inversion"]) if "inversion" in params else None
            
            try:
                target_angle = state["update_joint"](leg_idx, joint_idx, offset, control, inversion)
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "ok", "target_angle": target_angle}).encode("utf-8"))
            except Exception as e:
                self.send_response(500)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "error", "message": str(e)}).encode("utf-8"))
                
        elif path == "/save_offsets":
            try:
                status = state["save_offsets"]()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"status": status}).encode("utf-8"))
            except Exception as e:
                self.send_response(500)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "error", "message": str(e)}).encode("utf-8"))
        elif path == "/calibrate_adc":
            try:
                status = state["calibrate_adc"]()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"status": status}).encode("utf-8"))
            except Exception as e:
                self.send_response(500)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "error", "message": str(e)}).encode("utf-8"))

class ReusableHTTPServer(http.server.HTTPServer):
    allow_reuse_address = True
    def __init__(self, server_address, RequestHandlerClass, state):
        self.state = state
        super().__init__(server_address, RequestHandlerClass)

def start_web_server(state, port=8080):
    # Bind strictly to localhost/127.0.0.1 for testing/local calibration security compliance
    # TODO(security): If remote access is explicitly required, allow binding to public interface
    server_address = ('127.0.0.1', port)
    httpd = ReusableHTTPServer(server_address, CalibrationHTTPRequestHandler, state)
    print(f"Starting RoboDog Calibration Web Server on port {port}...")
    print(f"Open your browser and navigate to http://127.0.0.1:{port}")
    try:
        threading.Thread(target=webbrowser.open, args=(f"http://127.0.0.1:{port}",), daemon=True).start()
    except Exception:
        pass
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down server.")
