"""Read-only verification of the final handoff ZIP and original workspace copies."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive',type=Path,default=ROOT/'manufacturing/FABRICATION_HANDOFF_A04_B05.zip')
    parser.add_argument('--check-workspace',action='store_true');args=parser.parse_args()
    with zipfile.ZipFile(args.archive) as bundle:
        names=bundle.namelist();assert len(names)==len(set(names)) and bundle.testzip() is None
        manifest=json.loads(bundle.read('handoff_manifest.json'))
        assert set(names)==set(manifest['archive_files_sha256'])|{'handoff_manifest.json'}
        assert all(not name.startswith(('/', '\\')) and '..' not in Path(name).parts for name in names)
        for name,digest in manifest['archive_files_sha256'].items():
            assert hashlib.sha256(bundle.read(name)).hexdigest()==digest,'Corrupt archive member '+name
        status=json.loads(bundle.read('preparation_status.json'))
        assert status['local_preparation_complete'] and status['final_visual_review'] and status['automatic_stages']==15
        assert not status['fabrication_release'] and not status['order_authorized'] and not status['order_placed']
        if args.check_workspace:
            for relative,digest in manifest['workspace_sources_sha256'].items():
                assert hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()==digest,'Stale workspace source '+relative
            rfq=ROOT/'manufacturing/RFQ_A04_B05_DRAFT_NOT_FOR_ORDER.zip'
            assert hashlib.sha256(rfq.read_bytes()).hexdigest()==manifest['original_RFQ_sha256']
        result=dict(status='PASS / final handoff integrity; no factory or purchase approval implied',
            members=len(names),workspace_checked=args.check_workspace,
            archive_sha256=hashlib.sha256(args.archive.read_bytes()).hexdigest(),
            local_preparation_complete=True,fabrication_release=False,order_placed=False)
    (ROOT/'manufacturing/handoff_integrity_validation.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
