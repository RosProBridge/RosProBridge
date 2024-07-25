#!/usr/bin/env python3
import sys
import os

sys.path.append(os.path.dirname(__file__))
from base import ProBridge

def main():
    cfg_path = sys.argv[1]
    node = ProBridge.start(cfg_path)

if __name__ == '__main__':
    main()