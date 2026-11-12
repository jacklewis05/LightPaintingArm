"""Start the light-painting server:  python -m lightpaint [--port 8765] [--no-browser]"""
import argparse
import threading
import webbrowser

import uvicorn


def main():
    ap = argparse.ArgumentParser(description="Light-painting arm planner and live viewer")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--no-browser", action="store_true", help="don't open the viewer automatically")
    args = ap.parse_args()
    url = "http://127.0.0.1:%d" % args.port
    if not args.no_browser:
        threading.Timer(1.0, webbrowser.open, [url]).start()
    print("Light-painting viewer: %s  (Ctrl+C to quit)" % url)
    uvicorn.run("lightpaint.server:app", host="127.0.0.1", port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
