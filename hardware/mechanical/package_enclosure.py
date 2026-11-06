"""Package current, source-verified mechanical exports for Fusion and printing.

Standard Python; run after build_enclosure.py. This does not rebuild geometry.
"""
import hashlib
import json
from pathlib import Path
import zipfile

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
OUT=HERE/'exports'


def sha(data):return hashlib.sha256(data).hexdigest()


def main():
    params=json.loads((HERE/'enclosure_parameters.json').read_text(encoding='utf-8'))
    tag=params['export_tag']
    source=OUT/('source_manifest_'+tag+'.json')
    model=json.loads(source.read_text(encoding='utf-8'))
    for relative,digest in model['source_sha256'].items():
        assert sha((ROOT/relative).read_bytes())==digest,'Stale mechanical source: '+relative
    validation=json.loads((OUT/'validation.json').read_text(encoding='utf-8'))
    sequence=json.loads((OUT/'assembly_sequence_validation.json').read_text(encoding='utf-8'))
    assert not validation['collisions_mm3'] and not sequence['failures']
    assert model['printed_parts']==validation['printed_parts_STEP']['solid_count']
    assert model['assembly_service_stages']==len(sequence['tests'])
    files={}
    def add(path):
        name=path.relative_to(ROOT).as_posix()
        assert name not in files
        files[name]=path.read_bytes()
    for relative in model['source_sha256']:add(ROOT/relative)
    for path in (HERE/'README.md',HERE/'package_enclosure.py',ROOT/'display_backlight_study.md',
                 ROOT/'manufacturing/power_and_usb_review.md'):add(path)
    for name in ('enclosure_'+tag+'_integrated_buck.step','enclosure_'+tag+'_printed_parts.step',
                 'enclosure_exploded.step','enclosure_printed_parts_exploded.step',
                 'source_manifest_'+tag+'.json','validation.json','assembly_sequence_validation.json'):
        add(OUT/name)
    for path in sorted((OUT/('parts_'+tag)).iterdir()):
        if path.suffix in ('.step','.stl'):add(path)
    for path in sorted(OUT.glob('preview_*.png')):
        # Old external-buck preview is historical, not part of the current model.
        if path.name!='preview_buck.png':add(path)
    files['READ_FIRST.txt']=(
        f'Enclosure {tag}: {model["printed_parts"]} print pieces, {model["assembled_STEP_solids"]} reference solids.\n'
        f'Open mechanical/exports/enclosure_{tag}_integrated_buck.step in Fusion.\n'
        'Printed parts and individual STEP/STL are separate; components are clearance references.\n'
        'Window insert: translucent material. Cassette and other parts: opaque.\n'
        'BH1750 module dimensions, optical transmission, clips, cables and thermal behavior need physical checks.\n'
        'Sources are parametric CadQuery/Python; STEP does not carry native Fusion feature history.\n'
        'See mechanical/README.md and display_backlight_study.md.\n').encode('utf-8')
    manifest=dict(revision=params['revision'],files_sha256={n:sha(b) for n,b in sorted(files.items())},
                  geometry_source_manifest=source.relative_to(ROOT).as_posix(),
                  physical_fit_qualified=False,optical_transmission_qualified=False)
    files['bundle_manifest.json']=(json.dumps(manifest,indent=2)+'\n').encode('utf-8')
    archive=OUT/('enclosure_'+tag+'_integrated_buck_fusion_bundle.zip')
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as bundle:
        for name,data in sorted(files.items()):bundle.writestr(name,data)
    with zipfile.ZipFile(archive) as bundle:
        assert bundle.testzip() is None and len(bundle.namelist())==len(files)
        for name,digest in manifest['files_sha256'].items():assert sha(bundle.read(name))==digest
    (OUT/(archive.name+'.sha256')).write_text(sha(archive.read_bytes())+'  '+archive.name+'\n',encoding='ascii')
    print(json.dumps(dict(archive=str(archive),members=len(files),integrity='PASS'),indent=2))


if __name__=='__main__':main()
