"""Independently check the RFQ ZIP, hashes, CSV scope and release status.

Standard Python, read-only. Integrity verification never grants order approval.
"""
import csv
import hashlib
import io
import json
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parents[1]
FOLDER=ROOT/'manufacturing'


def main():
    archive=FOLDER/'RFQ_A04_B05_DRAFT_NOT_FOR_ORDER.zip'
    with zipfile.ZipFile(archive) as bundle:
        names=bundle.namelist()
        assert len(names)==len(set(names)), 'Duplicate archive members'
        assert bundle.testzip() is None, 'ZIP CRC failure'
        manifest=json.loads(bundle.read('manifest.json'))
        expected={'manifest.json'} | set(manifest['output_sha256']) | set(manifest['attachments_sha256'])
        assert set(names)==expected, ('Untracked or missing members',set(names)^expected)
        for section in ('output_sha256','attachments_sha256'):
            for name,digest in manifest[section].items():
                assert hashlib.sha256(bundle.read(name)).hexdigest()==digest, 'Archive hash mismatch '+name
        for name,digest in manifest['sources_sha256'].items():
            assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest, 'Stale native/review source '+name
        release=json.loads(bundle.read('release_checklist.json'))
        assert release['fabrication_release'] is False and release['order_placed'] is False
        assert manifest['status'].startswith('DRAFT')
        def rows(name):
            return list(csv.DictReader(io.StringIO(bundle.read(name).decode('utf-8-sig'))))
        counts={}
        for code in ('A','B'):
            bom=rows(code+'/BOM_SMT_RFQ_NOT_FOR_ORDER.csv')
            cpl=rows(code+'/CPL_SMT_RFQ_ROTATIONS_UNVERIFIED.csv')
            refs={r['Designator'] for r in bom}
            assert len(refs)==len(bom)==len(cpl)
            assert refs=={r['Designator'] for r in cpl}
            assert 'NT1' not in refs
            if code=='A': assert 'U1' not in refs
            tht=rows(code+'/THT_assembly_quote.csv')
            counts[code]=dict(SMT=len(bom),THT_native=len(tht),
                unmatched_SMT_supplier_rows=sum(not r['LCSC Part #'] for r in bom))
        sockets=rows('A/U1_socket_assembly_quote.csv')
        assert len(sockets)==2 and {r['Assembly_item'] for r in sockets}=={'U1_SOCKET_BOTTOM','U1_SOCKET_TOP'}
        tht=rows('assembly_review/assembly_THT_placement.csv')
        assert len(tht)==19
        assert len({(r['Board'],r['Assembly_item']) for r in tht})==19
        assert sum(b['SMT'] for b in counts.values())==51
        assert sum(b['THT_native'] for b in counts.values())+len(sockets)==19
        geometry=json.loads(bundle.read('assembly_review/assembly_geometry.json'))
        assert sum(len(b['parts']) for b in geometry['boards'])==70
        pdf=bundle.read('assembly_review/assembly_review_A04_B05.pdf')
        assert pdf.startswith(b'%PDF-')
        result=dict(status='PASS / RFQ integrity only; not fabrication release',
            archive=archive.name,archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
            members=len(names),source_hashes_checked=len(manifest['sources_sha256']),
            boards=counts,physical_THT_items=19,drawings_pages=manifest['assembly_drawings']['pages'],
            fabrication_release=False,order_placed=False)
    (FOLDER/'rfq_integrity_validation.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
