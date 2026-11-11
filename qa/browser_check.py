"""Local browser QA through Chrome DevTools, using an isolated Chrome profile."""
import base64,json,time,urllib.request
from pathlib import Path
import websocket
ROOT=Path(__file__).resolve().parents[1]
BASE='http://127.0.0.1:8769'
targets=json.load(urllib.request.urlopen('http://127.0.0.1:9338/json'))
target=next(x for x in targets if x['type']=='page')
ws=websocket.create_connection(target['webSocketDebuggerUrl'],origin='http://localhost:9338',timeout=60)
next_id=0; errors=[]
def call(method,params=None):
 global next_id
 next_id+=1;ident=next_id;ws.send(json.dumps({'id':ident,'method':method,'params':params or {}}))
 while True:
  msg=json.loads(ws.recv())
  if msg.get('method')=='Runtime.exceptionThrown':errors.append(msg['params'])
  if msg.get('id')==ident:
   if 'error' in msg:raise RuntimeError(msg['error'])
   return msg.get('result',{})
def js(expression):
 r=call('Runtime.evaluate',{'expression':expression,'returnByValue':True,'awaitPromise':True,'userGesture':True})
 if 'exceptionDetails' in r:raise RuntimeError(r['exceptionDetails'])
 return r.get('result',{}).get('value')
def wait(expression,timeout=45):
 until=time.monotonic()+timeout
 while time.monotonic()<until:
  if js(expression):return
  time.sleep(.1)
 raise AssertionError('Timed out: '+expression)
def shot(name):
 data=call('Page.captureScreenshot',{'format':'png','captureBeyondViewport':False})['data']
 (ROOT/'qa'/name).write_bytes(base64.b64decode(data))
call('Page.enable');call('Runtime.enable')
call('Emulation.setDeviceMetricsOverride',{'width':1440,'height':1000,'deviceScaleFactor':1,'mobile':False})
call('Page.navigate',{'url':BASE})
wait("document.readyState==='complete' && document.querySelector('#lesson-code').textContent.includes('buildModelFromUrdf')")
wait("document.querySelector('#robot-canvas').dataset.ready==='true'")
assert js('document.documentElement.scrollWidth<=innerWidth')
assert js("document.querySelector('#damping').value")=='0.03'
shot('desktop.png')
js("document.querySelector('#solve-btn').click()")
wait("document.querySelector('#solver-status').textContent==='Target reached'")
assert js("document.querySelector('#iteration-value').textContent")=='9'
assert abs(float(js("document.querySelector('#error-value').textContent"))-.069)<.002
js("document.querySelector('#focus-btn').click(); document.querySelector('#show-frames').click()")
assert js("document.querySelector('#focus-btn').getAttribute('aria-pressed')")=='true'
time.sleep(.3);shot('arm-focus.png')
js("document.querySelector('#zoom-in').click();document.querySelector('#zoom-out').click();document.querySelector('#camera-reset').click();document.querySelector('#show-frames').click();document.querySelector('#show-path').click()")
assert js("document.querySelector('#focus-btn').getAttribute('aria-pressed')")=='false'
js("document.querySelector('#reset-btn').click(); document.querySelector('#step-btn').click()")
assert js("document.querySelector('#iteration-value').textContent")=='1'
js("document.querySelector('[data-preset=higher]').click(); document.querySelector('#solve-btn').click()")
wait("document.querySelector('#solver-status').textContent==='Target reached'")
js("document.querySelector('[data-preset=far]').click(); for(let i=0;i<401;i++){document.querySelector('#step-btn').click();if(['No improvement','Iteration limit'].includes(document.querySelector('#solver-status').textContent))break;}")
assert js("document.querySelector('#solver-status').textContent") in ['No improvement','Iteration limit']
assert float(js("document.querySelector('#error-value').textContent"))>1
js("document.querySelector('#reset-btn').click()")
for i in range(6):
 js(f"document.querySelector('[data-lesson=\"{i}\"]').click()")
 assert js("document.querySelector('#lesson-counter').textContent")==f'STEP {i+1:02d} / 06'
 assert len(js("document.querySelector('#lesson-code').textContent"))>100
js("document.querySelector('#build').scrollIntoView();")
time.sleep(.3);shot('implementation.png')
js("document.querySelector('#open-code').click()")
wait("document.querySelector('#code-dialog').open && document.querySelector('#full-code').textContent.includes('def solve_ik')")
js("document.querySelector('#close-code').click()")
assert not js("document.querySelector('#code-dialog').open")
for name in ['mac','linux','windows']:
 js(f"document.querySelector('[data-os={name}]').click()")
 assert js(f"document.querySelector('[data-os={name}]').getAttribute('aria-selected')")=='true'
for path in ['downloads/g1-left-arm-pinocchio.zip','downloads/g1-full-lab.zip']:
 with urllib.request.urlopen(BASE+'/'+path) as r:assert r.read(4)==b'PK\x03\x04'
call('Browser.grantPermissions',{'origin':BASE,'permissions':['clipboardReadWrite','clipboardSanitizedWrite']})
call('Emulation.setFocusEmulationEnabled',{'enabled':True})
js("document.querySelector('#copy-step').click()")
wait("document.querySelector('#toast').textContent==='Copied to clipboard'")
assert 'pin.integrate' in js('navigator.clipboard.readText()')
call('Emulation.setDeviceMetricsOverride',{'width':390,'height':844,'deviceScaleFactor':1,'mobile':True})
js('window.scrollTo(0,0)');time.sleep(.4)
assert js('document.documentElement.scrollWidth<=innerWidth')
shot('mobile.png')
js("document.querySelector('#playground').scrollIntoView();document.querySelector('#focus-btn').click()")
time.sleep(.3);shot('mobile-viewer.png')
js("document.querySelector('#build').scrollIntoView()")
time.sleep(.3);shot('mobile-implementation.png')
assert js('document.documentElement.scrollWidth<=innerWidth')
assert not errors,errors
print('PASS: WebGL mesh loading, focus, frames, zoom and camera reset, desktop/mobile layout, default and high reach, unreachable result, one-step, six lessons, full-code dialog, OS tabs, clipboard and both downloads. No JS exceptions.')
ws.close()
