"""Read public JLCPCB catalogue pages, with no login, purchase or file upload.

Parse JSON embedded by Next.js without executing remote scripts. Do not infer
inventory from a product card, related parts, price tiers or search snippets.
Identity verification is intentionally separate from purchasing approval.
"""
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import time
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
FOLDER=ROOT/'manufacturing'


def norm(value):
    return re.sub(r'[^a-z0-9]', '', value.lower())


def fetch(mpn, code, expected_brand):
    url='https://jlcpcb.com/partdetail/'+code
    result=dict(requested_mpn=mpn, supplier_id=code, source_url=url,
                checked_utc=datetime.now(timezone.utc).isoformat(),
                identity_verified=False, stock_quantity=None,
                stock_status='NOT EXPOSED / verify in assembly quotation', purchase_approved=False)
    try:
        with urllib.request.urlopen(url,timeout=30) as response:
            data=response.read()
            result['resolved_url']=response.url
        html=data.decode('utf-8')
        result['public_page_sha256']=hashlib.sha256(data).hexdigest()
        payload=''.join(json.loads(x) for x in re.findall(
            r'self\.__next_f\.push\(\[1,("(?:[^"\\]|\\.)*")\]\)',html))
        records={}
        decoder=json.JSONDecoder()
        for match in re.finditer(r'(?m)^([0-9a-f]+):',payload):
            try:
                records[match[1]]=decoder.raw_decode(payload[match.end():])[0]
            except json.JSONDecodeError:
                pass
        # Flight text frames are length-prefixed, so a following JSON record
        # need not begin at a newline. Also inspect explicit product objects;
        # distinguish the main product from SEO summaries and related cards.
        objects=[]
        for match in re.finditer(r'\{"(?:componentModelEn|componentCode|componentPriceKeyId)"',payload):
            try:
                objects.append(decoder.raw_decode(payload[match.start():])[0])
            except json.JSONDecodeError:
                pass
        details=list({json.dumps(d,sort_keys=True):d for d in objects if isinstance(d,dict)
                      and d.get('componentCode')==code and 'assemblyMode' in d}.values())
        identity_keys=('componentCode','componentModelEn','componentBrandEn','componentSpecificationEn',
                       'assemblyMode','componentLibraryType','moistureSensitivityLevelEn','dataManualUrl')
        identities={tuple(d.get(key) for key in identity_keys) for d in details}
        if len(identities)!=1:
            raise ValueError('Main product identity missing or conflicting')
        detail=details[0]
        actual_mpn=detail['componentModelEn']
        brand=detail['componentBrandEn']
        aliases={'texasinstruments':{'ti','texasinstruments'}, 'diodesincorporated':{'diodes','diodesincorporated'},
                 'kemetyageogroup':{'kemet','yageogroup','kemetyageogroup'},
                 'stmicroelectronics':{'st','stmicroelectronics'}, 'vishaydale':{'vishay','vishaydale','vishayintertech'},
                 'vishay':{'vishay','vishayintertech'}, 'infineon':{'infineon','infineontechnologies'},
                 'walsin':{'walsin','walsintechcorp'}}
        acceptable=aliases.get(norm(expected_brand),{norm(expected_brand)})
        if norm(actual_mpn)!=norm(mpn) or norm(brand) not in acceptable:
            raise ValueError('Identity mismatch: '+brand+' '+actual_mpn)
        result.update(identity_verified=True,mpn=actual_mpn,manufacturer=brand,
                      package=detail.get('componentSpecificationEn'),
                      assembly_mode=detail.get('assemblyMode'),library_type=detail.get('componentLibraryType'),
                      MSL=detail.get('moistureSensitivityLevelEn'),
                      supplier_datasheet_url=detail.get('dataManualUrl'))
        # Stock must belong to the verified main record, not to a carousel.
        if isinstance(detail.get('stockCount'),int):
            result.update(stock_quantity=detail['stockCount'],stock_status='main record quantity observed; not reserved')
        tiers={d['startNumber']:dict(min_qty=d['startNumber'],max_qty=d['endNumber'],
            public_productPrice=d['productPrice']) for d in objects if isinstance(d,dict)
            and d.get('componentCode')==code and 'startNumber' in d and 'productPrice' in d}
        result['reference_price_tiers']=[tiers[n] for n in sorted(tiers)]
        result['price_scope']='Public raw productPrice fields only; currency/tax/assembly fees/stock not established as an order quotation.'
    except Exception as exc:
        result['error']=type(exc).__name__+': '+str(exc)
        result['http_status']=getattr(exc,'code',None)
        if result['http_status']==429:
            result['retry_after']=exc.headers.get('Retry-After')
    return result


def main():
    candidates=json.loads((FOLDER/'supplier_candidates.json').read_text(encoding='utf-8'))
    selected={}
    for code in ('A','B'):
        path=FOLDER/'review_A04_B05'/code/'procurement_review.csv'
        with path.open(encoding='utf-8-sig',newline='') as stream:
            for row in csv.DictReader(stream):
                if row['MPN']:
                    selected.setdefault(row['MPN'],dict(manufacturer=row['Manufacturer'],refs=[]))['refs'].append(code+'/'+row['Reference'])
    socket=json.loads((ROOT/'module_selection.json').read_text(encoding='utf-8'))['devkit']['socket']
    selected[socket['MPN']]=dict(manufacturer=socket['Manufacturer'],refs=['A/U1_SOCKET_BOTTOM','A/U1_SOCKET_TOP'])
    tasks=[(mpn,code,selected[mpn]['manufacturer']) for mpn,code in candidates['parts'].items()]
    records=[]
    rate_limited=False
    for task in tasks:
        if rate_limited:
            records.append(dict(requested_mpn=task[0],supplier_id=task[1],
                source_url='https://jlcpcb.com/partdetail/'+task[1],identity_verified=False,
                stock_quantity=None,stock_status='NOT CHECKED / prior HTTP429; retry later',
                purchase_approved=False,error='Not requested after rate limiting'))
            continue
        records.append(fetch(*task))
        rate_limited=records[-1].get('http_status')==429
        if not rate_limited:
            time.sleep(1.2)
    for row in records:
        row['board_references']=selected[row['requested_mpn']]['refs']
    verified=sum(r['identity_verified'] for r in records)
    result=dict(scope='Public catalogue identity/assembly classification only; no order, reservation or upload',
                checked_date=datetime.now(timezone.utc).date().isoformat(), verified_exact_MPNs=verified,
                live_stock_confirmed_MPNs=sum(r['stock_quantity'] is not None for r in records),
                candidate_input_sha256=hashlib.sha256((FOLDER/'supplier_candidates.json').read_bytes()).hexdigest(),
                records=records, unresolved_exact_MPNs=candidates['unmatched_exact_MPNs'])
    (FOLDER/'supplier_review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    with (FOLDER/'supplier_review.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        fields=['MPN','Board_references','JLCPCB_ID','Identity_verified','Manufacturer','Package','Assembly_mode','Stock','Source','Issue']
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for row in records:
            writer.writerow(dict(MPN=row['requested_mpn'],Board_references=' '.join(row['board_references']),
                JLCPCB_ID=row['supplier_id'],Identity_verified=row['identity_verified'],Manufacturer=row.get('manufacturer',''),
                Package=row.get('package',''),Assembly_mode=row.get('assembly_mode',''),
                Stock=row['stock_status'],Source=row['source_url'],Issue=row.get('error','')))
        for mpn,note in candidates['unmatched_exact_MPNs'].items():
            writer.writerow(dict(MPN=mpn,Board_references=' '.join(selected.get(mpn,{}).get('refs',[])),
                Identity_verified=False,Stock='UNMATCHED / sourcing quotation needed',Issue=note))
    print(json.dumps(dict(verified=verified,total=len(records),unmatched=len(candidates['unmatched_exact_MPNs']),
                          issues=[r.get('error') for r in records if not r['identity_verified']]),indent=2))


if __name__=='__main__':
    main()
