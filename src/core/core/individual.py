"""Optional isolated mission runs; stop at completion instead of advancing."""
import sys
from .core import main


def run(stage):
    main([*sys.argv[1:], '--ros-args', '-p', f'start_stage:={stage}',
          '-p', 'stop_after_stage:=true'])


def slow(): run(1)
def guide(): run(2)
def pedestrian(): run(3)
def obstacle(): run(4)
def intersection(): run(5)
def cone(): run(6)
def traffic(): run(7)
def parking(): run(8)
