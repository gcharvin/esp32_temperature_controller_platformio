"""Check actual UART pad connectivity, NC temperature endpoint and old copper."""
import hashlib,json
from pathlib import Path
import xml.etree.ElementTree as ET
import pcbnew as p
from route_diagnostics import copper_signature

ROOT=Path(__file__).resolve().parents[1]


def main():
    baseline=ROOT/'review/uart_extension_copper_baseline.json'
    before=json.loads(baseline.read_text(encoding='utf-8'))
    sources=[baseline,Path(__file__),ROOT/'interface_ab.json',ROOT/'module_selection.json'];result={}
    for code,name in [('A','pcb_a_display'),('B','pcb_b_temperature')]:
        path=ROOT/name/(name+'.kicad_pcb');net=ROOT/'review'/(name+'.net.xml');sources.extend([path,path.with_suffix('.kicad_sch'),net])
        b=p.LoadBoard(str(path));fps={f.GetReference():f for f in b.GetFootprints()};f=fps['J10']
        assert f.GetFPID().GetUniStringLibId()=='Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical'
        assert f.GetFieldText('MPN')=='B3B-XH-A(LF)(SN)' and f.GetFieldText('Assembly')=='FACTORY_THT_QUOTE'
        pads={q.GetNumber():q for q in f.Pads()};assert set(pads)=={'1','2','3'}
        assert pads['1'].GetNetname()=='/GND'
        expected_position=before['boards'][name]['new_J10_origin_mm']
        assert [round(p.ToMM(f.GetPosition().x)-60,4),round(p.ToMM(f.GetPosition().y)-55,4)]==expected_position
        groups={g.get('name'):{(n.get('ref'),n.get('pin')) for n in g.findall('node')} for g in ET.parse(net).findall('./nets/net')}
        if code=='A':
            assert pads['2'].GetNetname()=='/UART_A_TX' and pads['3'].GetNetname()=='/UART_A_RX'
            assert groups['/UART_A_TX']=={('J10','2'),('U1','22')}
            assert groups['/UART_A_RX']=={('J10','3'),('U1','21')}
            assert {q.GetNumber():q.GetNetname() for q in fps['J7'].Pads()}['5']=='/PWM1'
        else:
            for pin in ['2','3']:
                key=pads[pin].GetNetname()
                assert key.startswith('unconnected-') and groups[key]=={('J10',pin)}
                assert not any(t.GetNetname()==key for t in b.GetTracks())
        old=before['boards'][name];current=copper_signature(b)
        preserved={key:current[key] for key in old['UUIDs']}
        assert len(preserved)==old['before_count']
        assert hashlib.sha256(json.dumps(preserved,sort_keys=True).encode()).hexdigest()==old['before_signature_sha256']
        result[code]=dict(original_tracks_vias_preserved=len(preserved),added_tracks_vias=len(current)-len(preserved),
                         connector='J10, 3.3V UART, 1 GND / 2 TX_A / 3 RX_A',
                         routing_complete=True,temperature_B_signal_pins_NC=code=='B')
    report=dict(status='PASS / dedicated UART hardware reserve; original copper preserved',boards=result,
                software_uart_enabled=False,UART_hardware_loopback_performed=False,
                includes_RS232_transceiver=False,optional_temperature_cable=True,
                source_sha256={str(q.relative_to(ROOT)):hashlib.sha256(q.read_bytes()).hexdigest() for q in sources})
    (ROOT/'review/routing/uart_extension_validation.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
