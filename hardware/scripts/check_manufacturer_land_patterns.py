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
    paths = [source, review, placement, Path(__file__), lib/(spec['footprint'].split(':')[1]+'.kicad_mod')]
    result = dict(status='PASS / selected dimensions only', checks=['R13 native and library recommended lands',
        'R13 capacitor-return topology', 'C8/C10 footprint and lead pitch; distinguishable positive pad'],
        source_sha256={path.relative_to(ROOT).as_posix():hashlib.sha256(path.read_bytes()).hexdigest() for path in paths},
        limits=['Other footprints, stencil and factory approval are outside this check.'])
    (ROOT/'review/routing/manufacturer_land_patterns.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
