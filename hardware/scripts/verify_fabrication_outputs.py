"""Independently compare actual Excellon/Gerber files with native PCB geometry.

KiCad Python. Checks current decimal-metric, circular-hole, four-edge boards;
rejects unsupported formats. This is not a factory stencil/DFM approval.
"""
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re
import pcbnew as p

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'manufacturing/review_A04_B05'


def match_points(expected,actual,label,tolerance=.00011):
    assert len(expected)==len(actual),(label,'count',len(expected),len(actual))
    remaining=list(actual);maximum=0
    for category,x,y in expected:
        candidates=[(math.hypot(x-xx,y-yy),i) for i,(c,xx,yy) in enumerate(remaining) if c==category]
        assert candidates,(label,'missing category',category)
        error,i=min(candidates);assert error<=tolerance,(label,category,x,y,error)
        maximum=max(maximum,error);remaining.pop(i)
    assert not remaining
    return round(maximum,7)


def drills(path):
    text=path.read_text(encoding='utf-8')
    assert 'METRIC' in text and 'decimal' in text and 'G90' in text
    assert 'G85' not in text and 'INCH' not in text
    tools={};current=None;points=[]
    for line in text.splitlines():
        if m:=re.fullmatch(r'T(\d+)C([\d.]+)',line):tools[m[1]]=float(m[2])
        elif m:=re.fullmatch(r'T(\d+)',line):current=m[1]
        elif m:=re.fullmatch(r'X(-?[\d.]+)Y(-?[\d.]+)',line):
            assert current in tools
            points.append((round(tools[current],3),float(m[1]),float(m[2])))
    return points


def flashes(path):
    text=path.read_text(encoding='utf-8');ref=None;points=[]
    assert '%FSLAX46Y46*%' in text and '%MOMM*%' in text
    for line in text.splitlines():
        if m:=re.fullmatch(r'%TO.C,([^*]+)\*%',line):ref=m[1]
        elif line=='%TD*%':ref=None
        elif m:=re.fullmatch(r'X(-?\d+)Y(-?\d+)D03\*',line):
            assert ref is not None,'Unattributed paste flash'
            points.append((ref,int(m[1])/1e6,int(m[2])/1e6))
        elif 'D03*' in line:raise AssertionError('Unsupported paste coordinate '+line)
    assert 'G36*' not in text,'Unexpected region-based paste; review parser coverage'
    return points


def verify():
    result=dict(status='PASS / exported geometry checks; factory DFM still required',boards={},sources_sha256={},exports_sha256={},
        limits=['Circular decimal-metric drills and simple four-segment outlines only.',
                'Paste aperture positions/counts verified, not solder-paste volume or complete aperture geometry.',
                'Mask bridges, etching tolerance, thermal and assembly process need factory/bench validation.'])
    for code,name in [('A','pcb_a_display'),('B','pcb_b_temperature')]:
        native=ROOT/name/(name+'.kicad_pcb');board=p.LoadBoard(str(native));folder=OUT/code/'gerbers_DRAFT_NOT_FOR_ORDER'
        result['sources_sha256'][native.relative_to(ROOT).as_posix()]=hashlib.sha256(native.read_bytes()).hexdigest()
        expected=[];paste=[]
        for fp in board.GetFootprints():
            for pad in fp.Pads():
                pt=pad.GetPosition();x,y=p.ToMM(pt.x),-p.ToMM(pt.y)
                drill=pad.GetDrillSize()
                if drill.x:
                    assert drill.x==drill.y and pad.GetAttribute()==p.PAD_ATTRIB_PTH
                    expected.append((round(p.ToMM(drill.x),3),x,y))
                if pad.IsOnLayer(p.F_Paste):paste.append((fp.GetReference(),x,y))
        for via in board.GetTracks():
            if isinstance(via,p.PCB_VIA):
                pt=via.GetPosition();expected.append((round(p.ToMM(via.GetDrillValue()),3),p.ToMM(pt.x),-p.ToMM(pt.y)))
        plated=folder/(name+'-PTH.drl');nonplated=folder/(name+'-NPTH.drl')
        # KiCad's decimal metric Excellon rounds X/Y to0.001mm. Bound the
        # vector error by half a unit per axis, rather than demanding the
        # finer Gerber/native precision from a lower-resolution drill file.
        actual=drills(plated);drill_error=match_points(expected,actual,code+' drills',math.sqrt(2)*.0005+1e-8)
        assert not drills(nonplated)
        assert 'Plated,1,2,PTH' in plated.read_text() and 'NonPlated' in nonplated.read_text()
        edgefile=folder/(name+'-Edge_Cuts.gm1');edge=edgefile.read_text()
        assert '%TF.FileFunction,Profile,NP*%' in edge and '%FSLAX46Y46*%' in edge
        assert 'G02' not in edge and 'G03' not in edge
        current=None;segments=[]
        for x,y,mode in re.findall(r'X(-?\d+)Y(-?\d+)D0([12])\*',edge):
            point=(int(x)/1e6,int(y)/1e6)
            if mode=='1':
                assert current is not None;segments.append(tuple(sorted((current,point))))
            current=point
        expected_edges=[]
        for g in board.GetDrawings():
            if g.GetLayer()!=p.Edge_Cuts:continue
            assert g.GetShape()==p.SHAPE_T_SEGMENT
            expected_edges.append(tuple(sorted((tuple([p.ToMM(g.GetStart().x),-p.ToMM(g.GetStart().y)]),
                                                 tuple([p.ToMM(g.GetEnd().x),-p.ToMM(g.GetEnd().y)])))))
        assert len(segments)==4 and Counter(segments)==Counter(expected_edges),(code,'outline mismatch')
        actual_paste=flashes(folder/(name+'-F_Paste.gtp'))
        paste_error=match_points(paste,actual_paste,code+' paste')
        assert not flashes(folder/(name+'-B_Paste.gbp'))
        expected_SMT={ref for ref,_,_ in paste}
        assert len(expected_SMT)==(13 if code=='A' else 38)
        assert 'NT1' not in expected_SMT
        if code=='A':assert 'U1' not in expected_SMT
        required={'F_Cu.gtl':'Copper,L1,Top','B_Cu.gbl':'Copper,L2,Bot','F_Mask.gts':'Soldermask,Top',
                  'B_Mask.gbs':'Soldermask,Bot','F_Paste.gtp':'Paste,Top','B_Paste.gbp':'Paste,Bot',
                  'F_Silkscreen.gto':'Legend,Top','B_Silkscreen.gbo':'Legend,Bot','Edge_Cuts.gm1':'Profile,NP'}
        for suffix,function in required.items():
            file=folder/(name+'-'+suffix);text=file.read_text()
            assert '%TF.FileFunction,'+function+'*%' in text,(code,suffix,'layer identity')
            assert '%TF.SameCoordinates,Original*%' in text and '%MOMM*%' in text
        for path in sorted(folder.iterdir()):
            if path.is_file():result['exports_sha256'][path.relative_to(ROOT).as_posix()]=hashlib.sha256(path.read_bytes()).hexdigest()
        result['boards'][code]=dict(PTH_holes=len(actual),NPTH_holes=0,drill_diameters_mm=dict(Counter(str(d) for d,_,_ in actual)),
            maximum_drill_coordinate_error_mm=drill_error,drill_serialization_resolution_mm=.001,outline_segments=4,outline_matches_native=True,
            top_paste_apertures=len(actual_paste),SMT_designators_with_paste=len(expected_SMT),bottom_paste_apertures=0,
            maximum_paste_coordinate_error_mm=paste_error,nine_layer_identities_checked=True)
    result['sources_sha256'][Path(__file__).relative_to(ROOT).as_posix()]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (ROOT/'manufacturing/fabrication_output_validation.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    return result


if __name__=='__main__':print(json.dumps(verify(),indent=2))
