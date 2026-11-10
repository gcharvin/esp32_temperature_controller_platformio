"""Build the selected C10 maximum envelope as KiCad-compatible VRML2 meshes.

Units in a KiCad VRML model are 0.1 inch. Body dimensions come from the reviewed
manufacturer limits, not the nominal height of a generic D10 model.
"""
import math
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]


def cylinder(x, radius, bottom, top, color, segments=64):
    points=[((x+radius*math.cos(2*math.pi*i/segments))/2.54,
             radius*math.sin(2*math.pi*i/segments)/2.54,z/2.54)
            for z in (bottom,top) for i in range(segments)]
    faces=[]
    for i in range(segments):
        j=(i+1)%segments
        faces.extend([(i,j,j+segments),(i,j+segments,i+segments)])
    for i in range(1,segments-1):
        faces.extend([(0,i+1,i),(segments,segments+i,segments+i+1)])
    coords=',\n'.join(' '.join(f'{a:.8f}' for a in point) for point in points)
    indices=',\n'.join(','.join(map(str,face))+',-1' for face in faces)
    return f'''Shape {{
 appearance Appearance {{ material Material {{ diffuseColor {color} }} }}
 geometry IndexedFaceSet {{ solid FALSE creaseAngle 0.5
 coord Coordinate {{ point [ {coords} ] }}
 coordIndex [ {indices} ] }}
}}
'''


def main():
    dest=ROOT/'lab_hardware.3dshapes/EEUFS1V331LB_envelope.wrl'
    dest.parent.mkdir(exist_ok=True)
    text='#VRML V2.0 utf8\n# EEUFS1V331LB maximum envelope: body D8.5 H16.5 mm; P5 mm, pin1 at x0.\n'
    text+=cylinder(2.5,4.25,0,16.5,'0.13 0.15 0.17')
    for x in (0,5):text+=cylinder(x,.325,-3.2,0,'0.65 0.65 0.65',16)
    dest.write_text(text,encoding='ascii')
    print('Built C10 VRML mesh: D8.5 H16.5mm, P5; PCB-top origin; not detailed vendor CAD.')


if __name__=='__main__':main()
