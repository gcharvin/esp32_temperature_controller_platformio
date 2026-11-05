"""Render placement.json as a three-page dimensioned review PDF (reportlab)."""
import json
from pathlib import Path
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import landscape,A4
from reportlab.lib.colors import HexColor

ROOT=Path(__file__).resolve().parents[1]
DATA=json.loads((ROOT/'placement.json').read_text(encoding='utf-8'))
MM=72/25.4
W,H=landscape(A4)
OUT=ROOT/'review'/'implantation_v01.pdf'
C=canvas.Canvas(str(OUT),pagesize=(W,H))
C.setTitle('Implantation proposee PCB A et B et boitier - etude 0.1')
INK='#18313E';MUTED='#526672';BLUE='#DAEAF1';ORANGE='#FBE3C3';GREEN='#DDEADD';RED='#F2DBDC'

def text(x,y,txt,size=10,color=INK,bold=False):
    C.setFillColor(HexColor(color));C.setFont('Helvetica-Bold' if bold else 'Helvetica',size)
    C.drawString(x*MM,H-y*MM,txt)

def lines(x,y,values,size=9,step=5):
    for i,value in enumerate(values):text(x,y+i*step,value,size)

def box(x,y,w,h,fill=None,stroke=INK,dash=False):
    C.setStrokeColor(HexColor(stroke));C.setLineWidth(.6)
    C.setDash(3,2) if dash else C.setDash()
    if fill:C.setFillColor(HexColor(fill))
    C.rect(x*MM,H-(y+h)*MM,w*MM,h*MM,fill=bool(fill),stroke=True);C.setDash()

def line(x1,y1,x2,y2,color=INK):
    C.setStrokeColor(HexColor(color));C.setLineWidth(.6)
    C.line(x1*MM,H-y1*MM,x2*MM,H-y2*MM)

def dimension(x,y,w,label):
    line(x,y,x+w,y);line(x,y-1.5,x,y+1.5);line(x+w,y-1.5,x+w,y+1.5)
    text(x+w/2-8,y-2,label,9)

def header(title,sub,page):
    text(16,16,title,18,bold=True);text(16,24,sub,10,MUTED)
    line(16,29,281,29)
    text(16,200,'ETUDE 0.1 - COTES PROPOSEES - NON ROUTE - NE PAS FABRIQUER',9,MUTED)
    text(270,200,str(page)+'/3',9,MUTED)

def board_page(board,page):
    is_a=board['name']=='pcb_a_display'
    title='Carte A : facade et ESP32' if is_a else 'Carte B : temperature et puissance'
    width,height,thickness=board['board_mm'];scale=1.6 if is_a else 1.8
    ox,oy=20,54
    header(title,f'{width} x {height} mm | epaisseur proposee {thickness} mm | vue cote composants',page)
    box(ox,oy,width*scale,height*scale,fill='#F5F8FA')
    dimension(ox,oy-8,width*scale,f'{width} mm')
    text(ox-2,oy+height*scale+7,f'Hauteur {height} mm ; origine (0,0) en haut a gauche.',9)
    for x,y,w,h in board['clip_bands_mm']:
        box(ox+x*scale,oy+y*scale,w*scale,h*scale,fill=RED,stroke='#BC8C90')
    for x,y,w,h in board['clip_contact_patches_mm']:
        box(ox+x*scale,oy+y*scale,w*scale,h*scale,fill='#DDBABD',stroke='#BC8C90')
    if is_a:
        box(ox,oy+17*scale,20*scale,46*scale,fill='#E8E1F0',stroke='#9B89AE',dash=True)
    for row in board['components']:
        x,y,w,h=row['bounds_mm'];ref=row['reference'];placeholder=row['footprint'] is None
        fill=ORANGE if placeholder else (GREEN if ref.startswith('J') else BLUE)
        box(ox+x*scale,oy+y*scale,w*scale,h*scale,fill=fill,stroke='#687D87',dash=placeholder)
        if ref.startswith(('U','Q','J','F','D')):
            tx=ox+x*scale+1;ty=oy+y*scale+(h*scale/2)+1
            text(tx,ty,ref,8,bold=True)
            if placeholder and w>=25:text(tx,ty+4,'volume TBD',7.5)
    x=208
    lines(x,45,['Fixation sans vis'],11)
    lines(x,53,['Bandes haut/bas : 4 mm.',
                'Zones de contact : 12 x 4 mm.',
                'Appuis fixes + clips liberables.',
                'Pas de trous imposes a ce stade.'],9)
    lines(x,80,['Connecteurs vers arriere'],11)
    lines(x,88,['J1 : 5 V / JST-XH 2 points.',
                'J6 : I2C / JST-XH 3 points.',
                'J7 : commandes / XH 6 points.',
                'Degagement axial vise : 25 mm.',
                'Cables debranches hors tension.'],9)
    if is_a:
        lines(x,120,['DevKit : volume 60 x 32 mm.',
                     'Antenne au bord gauche.',
                     'USB : zone de service interne.',
                     'LCD et encodeur sur facade,',
                     'relies par cables a A.'],9)
    else:
        lines(x,120,['Puissance en partie haute.',
                     'ADC / sonde a droite.',
                     'Buck reserve : 30 x 22 mm.',
                     'B se retire vers arriere.',
                     'Dissipation a dimensionner.'],9)
    lines(x,155,['Legende'],10)
    box(x,159,5,3,ORANGE,dash=True);text(x+8,162,'TBD, sans pastilles',8)
    box(x,165,5,3,GREEN);text(x+8,168,'Empreinte connecteur',8)
    box(x,171,5,3,BLUE);text(x+8,174,'Empreinte CMS connue',8)
    text(20,191,'Les rectangles representent les encombrements au sol ; reperes R/C detailles dans KiCad.',9,MUTED)
    C.showPage()

def case_page():
    header('Boitier propose : deux logements independants','145 L x 100 H x 115 P mm exterieurs | parois 3 mm | schema mecanique de principe',3)
    # Front panel, x/y coordinates from placement.json.
    scale=.92;ox,oy=18,53
    text(ox,40,'Facade, vue de face',12,bold=True)
    box(ox,oy,145*scale,100*scale,fill='#F5F8FA');dimension(ox,oy-6,145*scale,'145 mm')
    box(ox+8*scale,oy+12*scale,98*scale,60*scale,fill=BLUE,dash=True)
    box(ox+18.5*scale,oy+29.4*scale,77*scale,25.2*scale,fill='#AFC9D5')
    text(ox+15*scale,oy+50*scale,'LCD : reserve 98 x 60',9)
    C.setFillColor(HexColor(GREEN));C.circle((ox+126*scale)*MM,H-(oy+45*scale)*MM,10*scale*MM,fill=True)
    text(ox+113*scale,oy+64*scale,'Encodeur',9)
    text(ox,oy+100*scale+7,'H = 100 mm ; decoupe LCD exacte a mesurer.',9)
    lines(18,162,['LCD 98 x 60 mm : exemple de format courant, pas une mesure de ton module.',
                  'Fenetre optique et bouton representes a titre indicatif.',
                  'Facade / LCD sur leur propre berceau, sans appui sur la DevKit.'],9)
    # Side section, d horizontally, case y vertically.
    ox,oy=175,53;s=.84
    text(ox,40,'Coupe laterale (profondeur)',12,bold=True)
    box(ox,oy,115*s,100*s,fill='#F5F8FA');dimension(ox,oy-6,115*s,'115 mm')
    box(ox+3*s,oy+12*s,25*s,60*s,fill=BLUE)
    text(ox+4*s,oy+42*s,'LCD',9)
    # Component face of A/B at 45/85 mm; laminate is toward the front.
    for depth,top,h,label,color in [(45,12,75,'A',GREEN),(85,18,65,'B',ORANGE)]:
        box(ox+(depth-1.6)*s,oy+top*s,1.6*s,h*s,fill=color)
        box(ox+depth*s,oy+(top+8)*s,20*s,(h-16)*s,fill=None,stroke='#8A9AA1',dash=True)
        text(ox+(depth-1)*s,oy+(top-3)*s,label,10,bold=True)
    box(ox+112*s,oy+3*s,3*s,94*s,fill='#C6D0D6')
    text(ox+77*s,oy+93*s,'AR amovible',8)
    # Extraction arrow.
    y=oy+50*s;line(ox+88*s,y,ox+126*s,y);line(ox+126*s,y,ox+122*s,y-2);line(ox+126*s,y,ox+122*s,y+2)
    lines(175,158,['Plans composants A/B a 45 / 85 mm.',
                   'Budget composants arriere : 20 mm.',
                   'Service JST : 25 mm, couvercle ouvert.',
                   'B extractible sans deposer A / facade.',
                   'Jeux et clips : essais imprimes requis.'],9)
    C.showPage()

for i,b in enumerate(DATA['boards'],1):board_page(b,i)
case_page();C.save()
print(OUT)
