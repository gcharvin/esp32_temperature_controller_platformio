"""Check selected manufacturer dimensions against the native B and local library.

Run with KiCad Python. This bounded check does not approve other footprints,
substitutions, stencil apertures, or a factory assembly process.
"""
import hashlib
import json
from pathlib import Path
import pcbnew as p

ROOT = Path(__file__).resolve().parents[1]


def main():
    source = ROOT/'pcb_b_temperature/pcb_b_temperature.kicad_pcb'
    review = ROOT/'manufacturing/land_pattern_review.json'
    specs = json.loads(review.read_text(encoding='utf-8'))
    board = p.LoadBoard(str(source))
    fps = {f.GetReference(): f for f in board.GetFootprints()}
    spec = specs['R13']
    lib = ROOT/'lab_hardware.pretty'
    library_fp = p.FootprintLoad(str(lib), spec['footprint'].split(':')[1])
    assert library_fp is not None
    for fp in (fps['R13'], library_fp):
        assert fp.GetFPID().GetUniStringLibId().split(':')[-1] == spec['footprint'].split(':')[-1]
        pads = {pad.GetNumber(): pad for pad in fp.Pads()}
        assert set(pads) == {'1', '2'}
        for pad in pads.values():
            assert [round(p.ToMM(v), 5) for v in (pad.GetSize().x, pad.GetSize().y)] == spec['pad_size_mm']
            assert pad.GetShape() == p.PAD_SHAPE_RECT
        a, b = (pads[n].GetPosition() for n in ('1', '2'))
        distance = p.ToMM(((b.x-a.x)**2+(b.y-a.y)**2)**0.5)
        assert abs(distance-spec['pad_center_distance_mm']) < 1e-6
        assert abs(distance-spec['pad_size_mm'][0]-spec['inner_gap_mm']) < 1e-6
        points = [pt for g in fp.GraphicalItems() if g.GetLayer() == p.F_CrtYd
                  for pt in (g.GetStart(), g.GetEnd())]
        dims = [round(p.ToMM(max(getattr(pt, axis) for pt in points)-min(getattr(pt, axis) for pt in points)), 5)
                for axis in ('x', 'y')]
        assert dims == spec['courtyard_mm']
    assert {pad.GetNumber(): pad.GetNetname() for pad in fps['R13'].Pads()} == {'1':'/BUCK_CAP_RETURN', '2':'/PGND'}
    placement=ROOT/'placement.json'
    row=next(row for b in json.loads(placement.read_text(encoding='utf-8'))['boards']
             if b['name']=='pcb_b_temperature' for row in b['components'] if row['reference']=='R13')
    assert row['height_mm'] >= spec['body_height_max_mm']
    assert row['footprint'] == spec['footprint']
    for ref in ('C8', 'C10'):
        fp = fps[ref]
        assert fp.GetFPID().GetUniStringLibId() == specs[ref]['footprint']
        pads = {pad.GetNumber(): pad for pad in fp.Pads()}
        a, b = (pads[n].GetPosition() for n in ('1', '2'))
        distance = p.ToMM(((b.x-a.x)**2+(b.y-a.y)**2)**0.5)
        assert abs(distance-specs[ref]['lead_pitch_mm']) < 1e-6
        assert pads['1'].GetShape() != pads['2'].GetShape(), 'Positive pad must remain distinguishable'
    apath=ROOT/'pcb_a_display/pcb_a_display.kicad_pcb';a=p.LoadBoard(str(apath))
    afps={fp.GetReference():fp for fp in a.GetFootprints()}
    host=afps['U1'];hspec=specs['A_U1_SOCKETS']
    assert host.GetFPID().GetUniStringLibId()==hspec['footprint']
    pads={pad.GetNumber():pad for pad in host.Pads()}
    assert set(pads)=={str(i) for i in range(1,31)}
    for pad in pads.values():
        assert abs(p.ToMM(pad.GetDrillSize().x)-hspec['drill_nominal_mm'])<1e-6
    assert abs(p.ToMM(pads['2'].GetPosition().x-pads['1'].GetPosition().x)-2.54)<1e-6
    assert abs(p.ToMM(pads['1'].GetPosition().y-pads['30'].GetPosition().y)-25.4)<1e-6
    led=specs['LED_APT2012']
    for code,refs in led['references'].items():
        for ref in refs:
            fp=(afps if code=='A' else fps)[ref]
            assert fp.GetFPID().GetUniStringLibId()==led['footprint']
            pads={pad.GetNumber():pad for pad in fp.Pads()}
            for pad in pads.values():
                assert [round(p.ToMM(v),5) for v in (pad.GetSize().x,pad.GetSize().y)]==led['pad_size_mm']
            v1,v2=(pads[n].GetPosition() for n in ('1','2'))
            assert abs(p.ToMM(((v2.x-v1.x)**2+(v2.y-v1.y)**2)**.5)-led['center_distance_mm'])<1e-6
    bspec=next(b for b in json.loads(placement.read_text(encoding='utf-8'))['boards'] if b['name']=='pcb_b_temperature')
    for ref in specs['B_J2_J3']['references']:
        fp=fps[ref];pads=list(fp.Pads());a1,a2=(pad.GetPosition() for pad in pads)
        assert abs(p.ToMM(((a2.x-a1.x)**2+(a2.y-a1.y)**2)**.5)-5.08)<1e-6
        assert all(abs(p.ToMM(pad.GetDrillSize().x)-1.3)<1e-6 for pad in pads)
        assert next(r for r in bspec['components'] if r['reference']==ref)['height_mm']>=13.8
    paths = [source, apath, review, placement, ROOT/'module_selection.json', Path(__file__),
             lib/(spec['footprint'].split(':')[1]+'.kicad_mod'),lib/(hspec['footprint'].split(':')[1]+'.kicad_mod'),
             lib/(led['footprint'].split(':')[1]+'.kicad_mod')]
    result = dict(status='PASS / selected dimensions only', checks=['R13 native and library recommended lands',
        'R13 capacitor-return topology', 'C8/C10 footprint and lead pitch; distinguishable positive pad',
        '30 socket holes1.05mm, contact pitch2.54mm and row spacing25.4mm', 'Six Kingbright LED land patterns',
        'Phoenix terminal pitch5.08mm, hole1.3mm, envelope at least installed height13.8mm'],
        source_sha256={path.relative_to(ROOT).as_posix():hashlib.sha256(path.read_bytes()).hexdigest() for path in paths},
        limits=['Other footprints, stencil and factory approval are outside this check.'])
    (ROOT/'review/routing/manufacturer_land_patterns.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
