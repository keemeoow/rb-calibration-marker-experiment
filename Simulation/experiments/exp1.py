#!/usr/bin/env python3
"""실험 1: Ours — FK 잔차보정 + 통합 + 큐브/보드

core(통합 엔진)에서 EXP1 설정을 가져와 실행한다.
  python experiments/exp1.py --seeds 20
"""
import sys, os, argparse, json
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from core import run_config, summarize
from configs import EXP1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--sigma_px", type=float, default=0.3)
    ap.add_argument("--dump", type=str, default=None)
    args = ap.parse_args()
    stats = run_config(EXP1, seeds=args.seeds, sigma_px=args.sigma_px)
    print(summarize(EXP1, stats))
    if args.dump:
        json.dump({"config": EXP1.__dict__, "stats": stats},
                  open(args.dump, "w"), indent=2, default=list)
        print(f"[저장] {args.dump}")


if __name__ == "__main__":
    main()
