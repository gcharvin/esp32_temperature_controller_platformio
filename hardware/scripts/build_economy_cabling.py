"""Build a reduced cable RFQ; keep internal budget out of the supplier ZIP."""
import argparse,csv,hashlib,html,json,zipfile
from pathlib import Path
from build_lcsc_cable_rfq import validate as validate_reference

ROOT=Path(__file__).resolve().parents[2]
F=ROOT/'hardware/manufacturing/economy_cabling'
PDF=F/'output/pdf/LCSC_CABLE_RFQ_ECONOMY.pdf'
ZIP=F.parent/'LCSC_CABLE_RFQ_ECONOMY_NOT_FOR_ORDER.zip'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def rows(n):return list(csv.DictReader((F/n).open(encoding='utf-8-sig',newline='')))

def validate():
    d=read(F/'economy_plan.json');old,pins=validate_reference()
    assert d['equipment_sets']==10 and d['user_soldering_accepted'] and not d['customer_crimping_required']
    assert len(d['custom'])==4 and len(d['allocations'])==13
    assert [h['id'] for h in d['custom']]==['H01','H02','H03','U04']
    assert [h['quantity'] for h in d['custom']]==[10,10,10,30]
    assert sum(h['quantity']+h['optional_extra_quantity'] for h in d['custom'])==70
    for h in d['custom'][:3]:
        original=next(x for x in old['harnesses'] if x['id']==h['id'])
        for k in ('end_a','end_b','awg','length_mm','tolerance_mm','map'):assert h[k]==original[k],(h['id'],k)
    u=d['custom'][-1]
    assert u['awg']==26 and u['end_a']=='XH4' and u['end_b']=='FLY4'
    assert [(r[0],r[1]) for r in u['map']]==[(str(i),str(i)) for i in range(1,5)]
    for pin,net in d['functional_map']['LCD_BH_TMP'].items():
        for ref in ('J2','J4','J5'):assert pins['A/'+ref+'.'+pin]==net
    for pin,net in d['functional_map']['ENCODER'].items():assert net.startswith(pins['A/J3.'+pin])
    assert pins['B/J4.1']=='+3V3_B' and pins['B/J4.2']=='NTC_RAW'
    assert len(rows('custom_quote_lines.csv'))==4 and len(rows('custom_pin_map.csv'))==15
    assert [int(r['Qty_base']) for r in rows('custom_quote_lines.csv')]==[10,10,10,30]
    assert [int(r['Qty_total_with_TMP']) for r in rows('custom_quote_lines.csv')]==[10,10,10,40]
    expected=[(h['id'],*r) for h in d['custom'] for r in h['map']]
    assert [tuple(r.values()) for r in rows('custom_pin_map.csv')]==expected
    assert len(rows('purchase_plan.csv'))==13 and len(rows('retail_basket.csv'))==len(d['retail'])
    known=sum(r['qty_base']*(r['unit_price_TTC_EUR'] or 0) for r in d['retail'])
    assert abs(known-81.6)<1e-8 and d['budget']['catalogue_public_base_TTC']==81.6
    b=d['budget']
    low=known+sum(b[k][0] for k in ['custom_base_ex_fees_tax_estimate','jack_lot_allocation_estimate','workshop_consumables_estimate','preparation_shipping_tax_contingency_estimate'])
    high=known+sum(b[k][1] for k in ['custom_base_ex_fees_tax_estimate','jack_lot_allocation_estimate','workshop_consumables_estimate','preparation_shipping_tax_contingency_estimate'])
    assert abs(low-b['resulting_base_cash_estimate'][0])<1e-8 and abs(high-b['resulting_base_cash_estimate'][1])<1e-8
    assert b['paid_soldering_labour'].startswith('EXCLUDED')
    return d,pins

def documents():
    from reportlab.lib import colors
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.styles import getSampleStyleSheet,ParagraphStyle
    from reportlab.platypus import SimpleDocTemplate,Paragraph,Spacer,Table,TableStyle,PageBreak
    d,pins=validate();PDF.parent.mkdir(parents=True,exist_ok=True)
    s=getSampleStyleSheet();s.add(ParagraphStyle(name='R',fontName='Helvetica',fontSize=10,leading=13,spaceAfter=8));s.add(ParagraphStyle(name='S',fontName='Helvetica',fontSize=8.5,leading=11,spaceAfter=5))
    def p(t,style='R'):return Paragraph(html.escape(str(t)),s[style])
    def table(data,widths):
        t=Table([[p(x,'S') for x in r] for r in data],colWidths=widths,hAlign='LEFT')
        t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#e7eef2')),('VALIGN',(0,0),(-1,-1),'TOP'),('BOTTOMPADDING',(0,0),(-1,-1),7),('TOPPADDING',(0,0),(-1,-1),7)]));return t
    story=[p('LCSC Cable - reduced RFQ','Title'),p('RFQ-CABLE-02-ECONOMY | 09 November2026'),p('QUOTATION ONLY - NO ORDER OR PRODUCTION RELEASE','Heading2'),
        p('Four standard cable types for10 laboratory instrument sets. Base60cables; alternate70cables with optionalTMP117. This current reduced request supersedes the13-line all-custom comparisonRFQ-CABLE-01.'),
        table([['Type','Base quantity','Alternate TOTAL with TMP']]+[[h['id'],h['quantity'],h['quantity']+h['optional_extra_quantity']] for h in d['custom']],[150,150,215]),
        p('U04 is one shared reference,30 OR40pieces, not30+40. Encoder and probe-jack soldering will be done locally. No custom encoder, jack, SH gauge transition, external probe, power, cutoff or USB subassemblies are included in this reduced supplier quote.'),
        p('No special common outer jacket or overmould requested for internal cables. Each wire remains insulated. Ordinary flexible stranded copper and simple cavity numbers are required. Quote any lower-cost compatible socket with exact manufacturer/MPN/drawing separately; approval is still required.'),
        p('Please itemize unit/setup/first-article/shipping/tax costs and lead time. No first article or production is authorized. Internal purchasing and estimated budgets are intentionally not attached to this supplier packet.'),PageBreak(),
        p('Common requirements and connector assignment','Heading1'),p(d['length_definition']),p(d['inspection']),
        p('XH2.50mm and independent header sockets2.54mm are distinct. JST XHP-2/3/4/6 + SXH-001T-P0.6 are the requested PCB-side family. U04 sockets use a documented Harwin M20-1060100 + M20-1160042 reference for0.64mm square male posts. Supplier alternatives need approval.'),
        p('Provide mating-face and wire-entry drawings, with cavity1 and key/latch. Pin maps are logical cavity numbers; do not mirror based on looking at the other end.'),
        table([['U04 pin','LCD / BH1750 / optional TMP','Encoder after local soldering']]+[[i,d['functional_map']['LCD_BH_TMP'][i],d['functional_map']['ENCODER'][i]] for i in ['1','2','3','4']],[75,205,235]),
        p('U04 carries only numbers1..4. Do NOT mark a fixedGND/VCC function on every U04: encoderGND is pin4, while I2C peripheralGND is pin1. Each flying socket must remain individually movable.'),
        p('Local catalogue probe pigtail, not supplied in this RFQ: B/J4.1 +3V3_B ->TIP; B/J4.2 NTC_RAW ->SLEEVE; ring isolated. SLEEVE IS NOT GND. No independent-cutoff requirement is waived by excluding it from this cable order.'),PageBreak()]
    for ix,h in enumerate(d['custom']):
        a=d['connectors'][h['end_a']];b=d['connectors'][h['end_b']]
        story += [p(h['id']+' - '+h['name'],'Heading1'),p('Base quantity '+str(h['quantity'])+' | alternate total '+str(h['quantity']+h['optional_extra_quantity'])+' | RFQ ONLY'),
            p('End A: '+h['from']+' | '+a['housing']+' | '+a['contact']),p('End B: '+h['to']+' | '+b['housing']+' | '+b['contact']),
            p('Wire '+str(h['awg'])+'AWG | L300+/-10mm between rear wire exits; supplier drawing must state exact datum'),
            table([['A cavity','B cavity/socket','Purpose','Colour / marking']]+h['map'],[70,95,160,190]),
            p('Standard construction','Heading2'),p(h['finish']),p('Approval points','Heading2'),p(h['approval']),
            p('No custom soldering or special jacket is requested. Preserve contact/wire compatibility and normal crimp/continuity quality.','S'),
            p('References: '+a['source']+' | '+b['source'],'S')]
        if ix<3:story.append(PageBreak())
    def footer(c,doc):
        c.setFont('Helvetica',8);c.drawString(40,21,'RFQ-CABLE-02-ECONOMY | QUOTATION ONLY | BASE60 / ALT70');c.drawRightString(555,21,str(doc.page))
    SimpleDocTemplate(str(PDF),pagesize=A4,leftMargin=40,rightMargin=40,topMargin=35,bottomMargin=42,title='LCSC Cable - four-type economy RFQ').build(story,onFirstPage=footer,onLaterPages=footer)
    print('Generated6-page economyRFQ: '+str(PDF))

def package():
    from pypdf import PdfReader
    d,pins=validate();assert len(PdfReader(PDF).pages)==6
    qa=read(F/'visual_review.json');assert qa['status']=='PASS' and qa['pdf_sha256']==sha(PDF)
    for n,v in qa['tables_sha256'].items():assert sha(F/n)==v,n
    source_paths=[ROOT/r for r in d['sources']]+[F/'economy_plan.json',Path(__file__),ROOT/'hardware/scripts/export_economy_cabling.mjs',ROOT/'hardware/scripts/build_lcsc_cable_rfq.py']
    report=dict(status='PASS_LOCAL_ECONOMY_PLAN',custom_types=4,base_custom_cables=60,with_TMP_custom_cables=70,allocation_rows=13,
        native_pin_entries_checked=len(pins),base_catalogue_public_TTC_EUR=81.6,paid_soldering_labour_included=False,
        catalogue_pigtail_slack_qualified=False,submitted=False,order_placed=False,physical_fit_qualified=False,
        source_sha256={p.relative_to(ROOT).as_posix():sha(p) for p in source_paths})
    (F/'validation.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    supplier=dict(revision=d['revision'],status='QUOTATION ONLY',custom=d['custom'],connectors=d['connectors'],
        length_definition=d['length_definition'],inspection=d['inspection'],base_total=60,alternate_with_TMP_total=70)
    (F/'supplier_specification.json').write_text(json.dumps(supplier,indent=2)+'\n',encoding='utf-8')
    files={n:(F/n).read_bytes() for n in ['request_quote_EN.txt','custom_quote_lines.csv','custom_pin_map.csv','supplier_specification.json',PDF.relative_to(F).as_posix()]}
    files['READ_FIRST.txt']=b'CURRENT RFQ-CABLE-02-ECONOMY:4types,60base or70alternate cables. Quotation only. No order authorized.\nU04 quantity30OR40total; not30+40. No custom soldered/jacketed assemblies in this quote.\n'
    manifest=dict(revision=d['revision'],submitted=False,order_placed=False,files_sha256={n:hashlib.sha256(v).hexdigest() for n,v in files.items()})
    (F/'package_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8');files['package_manifest.json']=(F/'package_manifest.json').read_bytes()
    assert not any('budget' in n.lower() or n.startswith('retail') for n in files)
    with zipfile.ZipFile(ZIP,'w',zipfile.ZIP_DEFLATED) as z:
        for n,v in sorted(files.items()):z.writestr(n,v)
    with zipfile.ZipFile(ZIP) as z:
        assert z.testzip() is None and len(z.namelist())==len(set(z.namelist()))==len(files)
        for n,v in files.items():assert z.read(n)==v
    ZIP.with_suffix('.zip.sha256').write_text(sha(ZIP)+'  '+ZIP.name+'\n',encoding='ascii')
    print(json.dumps(dict(status=report['status'],archive=str(ZIP),files=len(files),base60=True,alternate70=True,submitted=False),indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--documents',action='store_true');p.add_argument('--package',action='store_true');a=p.parse_args()
    if a.documents:documents()
    if a.package:package()
    if not(a.documents or a.package):validate();print('PASS: economy allocations, PCB contracts, quantities and budget arithmetic.')
