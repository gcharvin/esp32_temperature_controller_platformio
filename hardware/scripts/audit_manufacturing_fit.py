"""Check selected geometric factory constraints on the native boards, read-only.

This is a bounded DFM audit, not a manufacturer DFM approval or a replacement
for KiCad DRC. Component PTHs and vias have explicitly different rules.
"""
import hashlib
import json
from pathlib import Path

import pcbnew

ROOT=Path(__file__).resolve().parents[1]
FOLDER=ROOT/'manufacturing'


def main():
    policy=json.loads((FOLDER/'process_requirements.json').read_text(encoding='utf-8'))
    result=dict(status='limited geometric checks',sources=policy['sources'],boards={},
                limits=['Not factory DFM approval; pad solderability, mask bridges and full assembly preview still need review.',
                        'Minimum component-PTH rings and minimum vias are separate published constraints.',
                        'Proposed manufacturing rails are not functional PCB geometry and have not been accepted by the assembler.'])
    for code,name in [('A','pcb_a_display'),('B','pcb_b_temperature')]:
        path=ROOT/name/(name+'.kicad_pcb')
        board=pcbnew.LoadBoard(str(path))
        rules=policy['board_'+code]
        tracks=[pcbnew.ToMM(t.GetWidth()) for t in board.GetTracks() if not isinstance(t,pcbnew.PCB_VIA)]
        vias=[dict(x_mm=pcbnew.ToMM(t.GetPosition().x),y_mm=pcbnew.ToMM(t.GetPosition().y),
                   diameter_mm=pcbnew.ToMM(t.GetWidth(pcbnew.F_Cu)),drill_mm=pcbnew.ToMM(t.GetDrillValue()))
              for t in board.GetTracks() if isinstance(t,pcbnew.PCB_VIA)]
        pth=[]
        for fp in board.GetFootprints():
            for pad in fp.Pads():
                if pad.GetAttribute()!=pcbnew.PAD_ATTRIB_PTH:
                    continue
                size,drill,offset=pad.GetSize(),pad.GetDrillSize(),pad.GetOffset()
                ring=pcbnew.ToMM((min(size.x,size.y)-max(drill.x,drill.y))/2-max(abs(offset.x),abs(offset.y)))
                pth.append(dict(ref=fp.GetReference(),pad=pad.GetNumber(),annular_ring_lower_bound_mm=ring))
        checks=dict(track_width=min(tracks)+1e-8>=rules['min_track_width_mm'],
                    component_PTH_annular_ring=all(p['annular_ring_lower_bound_mm']+1e-8>=rules['PTH_component_min_annular_ring_mm'] for p in pth),
                    via_drill=all(v['drill_mm']>=policy['general']['min_via_drill_mm'] for v in vias),
                    via_diameter_margin=all(v['diameter_mm']-v['drill_mm']+1e-8>=policy['general']['min_via_diameter_minus_drill_mm'] for v in vias),
                    thickness=abs(pcbnew.ToMM(board.GetDesignSettings().GetBoardThickness())-1.6)<1e-8)
        result['boards'][code]=dict(native_PCB_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            track_min_mm=min(tracks),via_count=len(vias),
            via_min_annular_ring_mm=min((v['diameter_mm']-v['drill_mm'])/2 for v in vias),
            component_PTH_count=len(pth),component_PTH_min_ring_mm=min(p['annular_ring_lower_bound_mm'] for p in pth),
            checks=checks,failures=[p for p in pth if p['annular_ring_lower_bound_mm']+1e-8<rules['PTH_component_min_annular_ring_mm']])
    result['all_checked_constraints_passed']=all(all(b['checks'].values()) for b in result['boards'].values())
    result['policy_sha256']=hashlib.sha256((FOLDER/'process_requirements.json').read_bytes()).hexdigest()
    (FOLDER/'manufacturing_fit_audit.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,indent=2))
    if not result['all_checked_constraints_passed']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
