"""Offline entrypoint. Importing/running this CLI never connects devices."""
import argparse,json
from .bridge import validate_bundle
def main(argv=None):
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('command',choices=['preflight']);p.add_argument('bundle');a=p.parse_args(argv)
 report=validate_bundle(a.bundle);print(json.dumps(report,indent=2));return 0 if report['local_bundle_complete']else 2
if __name__=='__main__':raise SystemExit(main())
