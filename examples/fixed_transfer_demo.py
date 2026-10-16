"""Run the complete certified simulation example; see --help for library/output paths."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from verify_fixed_transfer import main

if __name__=='__main__':
    raise SystemExit(main())
