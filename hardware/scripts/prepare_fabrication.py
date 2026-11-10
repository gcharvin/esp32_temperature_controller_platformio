"""Rebuild the complete local prototype-preparation evidence, fail on any error.

KiCad Python; --pdf-python points to a Python with ReportLab and pypdf.
Does not order, upload boards, contact a supplier, or flash any hardware.
Final visual review and the handoff builder run after these checks.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--pdf-python',type=Path,required=True);args=parser.parse_args()
    scripts=ROOT/'scripts';uv=Path.home()/'.local/bin/uv.exe'
    tag=json.loads((ROOT/'mechanical/enclosure_parameters.json').read_text(encoding='utf-8'))['export_tag']
    stages=[('schematics',[sys.executable,scripts/'check_schematics.py']),
            ('parts',[sys.executable,scripts/'check_component_selection.py']),
            ('lands',[sys.executable,scripts/'check_manufacturer_land_patterns.py']),
            ('routing',[sys.executable,scripts/'check_routing.py']),
            ('placement',[sys.executable,scripts/'check_placement.py']),
            ('test_points',[sys.executable,scripts/'check_test_points.py']),
            ('diagnostic_firmware',[args.pdf_python,scripts/'check_diagnostic_firmware.py']),
            ('buck_binding',[sys.executable,scripts/'check_integrated_buck.py']),
            ('factory_geometry',[sys.executable,scripts/'audit_manufacturing_fit.py']),
            ('harnesses',[sys.executable,scripts/'build_harness_map.py']),
            ('assembly_pdf',[sys.executable,scripts/'build_assembly_review.py','--pdf-python',args.pdf_python]),
            ('mechanical',[uv,'run',ROOT/'mechanical/build_enclosure.py']),
            ('routing_plots',[args.pdf_python,scripts/'render_routing_review.py']),
            ('manufacturing_exports',[sys.executable,scripts/'prepare_manufacturing_review.py']),
            ('rfq_integrity',[sys.executable,scripts/'verify_manufacturing_review.py'])]
    logs=Path(tempfile.mkdtemp(prefix='hardware-fabrication-'));done=[]
    for name,command in stages:
        print('RUN '+name,flush=True)
        result=subprocess.run(list(map(str,command)),capture_output=True,text=True)
        log=logs/(name+'.log');log.write_text(result.stdout+'\n'+result.stderr,encoding='utf-8')
        if result.returncode:
            print(result.stdout[-5000:]);print(result.stderr[-3000:]);raise RuntimeError(name+' failed; '+str(log))
        done.append(dict(stage=name,exit_code=0));print('PASS '+name,flush=True)
    files=[ROOT/'review/validation.txt',ROOT/'review/routing/component_selection_validation.json',ROOT/'review/routing/uart_extension_validation.json',
           ROOT/'review/routing/test_points_validation.json',ROOT/'review/diagnostic_firmware_validation.json',
           ROOT/'review/routing/manufacturer_land_patterns.json',ROOT/'review/routing/routing_validation.json',
           ROOT/'manufacturing/manufacturing_fit_audit.json',ROOT/'manufacturing/harness_validation.json',
           ROOT/'manufacturing/output/assembly_review_manifest.json',ROOT/'manufacturing/fabrication_output_validation.json',
           ROOT/'manufacturing/rfq_integrity_validation.json',ROOT/('mechanical/exports/source_manifest_'+tag+'.json'),
           ROOT/'mechanical/exports/assembly_sequence_validation.json',ROOT/'mechanical/exports/validation.json']
    result=dict(status='PASS / all15 local preparation stages; final visual review remains separate',stages=done,
        hardware_tests_performed=False,order_authorized=False,
        evidence_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        limits=['Factory sourcing/DFM/placement-preview/quotation approvals are external.',
                'No actual heater, DevKit power path or printed fit qualification inferred.'])
    (ROOT/'manufacturing/preparation_checks.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print('All local checks passed. Final image/PDF review and handoff packaging can proceed. Logs: '+str(logs),flush=True)


if __name__=='__main__':main()
