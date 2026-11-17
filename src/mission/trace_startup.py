"""Optional startup diagnostics for source-based mission processes."""
import faulthandler
from pathlib import Path
import runpy
import sys


def main():
    debug, target, *arguments = sys.argv[1:]
    enabled = debug.lower() == 'true'
    if enabled:
        print(f'[startup] {target}: loading imports; traceback in 10 seconds', flush=True)
        faulthandler.enable()
        faulthandler.dump_traceback_later(10, repeat=False)
        import rclpy
        from rclpy.node import Node

        original_init = rclpy.init
        original_node_init = Node.__init__

        def traced_init(*args, **kwargs):
            print('[startup] rclpy.init: begin', flush=True)
            result = original_init(*args, **kwargs)
            print('[startup] rclpy.init: complete', flush=True)
            return result

        def traced_node_init(self, *args, **kwargs):
            print(f'[startup] Node.__init__: begin {args}', flush=True)
            result = original_node_init(self, *args, **kwargs)
            print('[startup] Node.__init__: complete', flush=True)
            return result

        rclpy.init = traced_init
        Node.__init__ = traced_node_init
    sys.argv = [target, *arguments]
    sys.path.insert(0, str(Path(target).resolve().parent))
    try:
        runpy.run_path(target, run_name='__main__')
    finally:
        if enabled:
            faulthandler.cancel_dump_traceback_later()


if __name__ == '__main__':
    main()
