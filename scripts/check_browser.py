"""Optional UI smoke test: pip install playwright==1.62.0; use system Chromium."""
from pathlib import Path
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[1]
with sync_playwright() as p:
    browser=p.chromium.launch(executable_path='/usr/bin/chromium',headless=True,args=['--no-sandbox','--disable-dev-shm-usage'])
    page=browser.new_page(viewport={'width':1440,'height':1100},device_scale_factor=1)
    errors=[];failed_assets=[]
    page.on('pageerror',lambda e:errors.append(str(e)))
    page.on('response',lambda r:failed_assets.append((r.status,r.url)) if r.status>=400 and '/models/' in r.url else None)
    page.goto('http://127.0.0.1:8000',wait_until='networkidle')
    def ready():page.wait_for_function("!document.getElementById('run').disabled",timeout=90000)
    def select(s):page.locator('#scenario').select_option(s);ready()
    def run():
        page.get_by_role('button',name='Run experiment').click();ready()
        return page.locator('#status').inner_text()
    ready();assert 'converged' in run()
    page.locator('#play').click();page.wait_for_timeout(250);page.locator('#play').click()
    assert not page.locator('#frame-label').inner_text().startswith('0 /')
    page.locator('#scrubber').fill(page.locator('#scrubber').get_attribute('max'))
    page.screenshot(path=str(ROOT/'docs/workbench-desktop.png'),full_page=True)
    page.locator('#geometry-mode').select_option('visual');ready()
    assert 'updated' in page.locator('#status').inner_text()
    page.screenshot(path=str(ROOT/'docs/workbench-visual.png'),full_page=True)
    select('humanoid');assert 'converged' in run()
    select('obstacle');assert 'converged' in run();print('obstacle:',page.locator('#status').inner_text(),flush=True)
    page.screenshot(path=str(ROOT/'docs/workbench-obstacle.png'),full_page=True)
    select('unitree-g1');assert page.locator('#dof').inner_text()=='29'
    page.screenshot(path=str(ROOT/'docs/workbench-g1.png'),full_page=True)
    page.locator('#geometry-mode').select_option('collision');ready()
    # Custom URDF upload, function generation, and pose-target solve.
    page.locator('#upload').set_input_files(str(ROOT/'models/panda.urdf'));ready()
    page.locator('#custom-link').select_option('panda_link8')
    page.locator('#method').select_option('reduced-osqp');assert 'converged' in run()
    select('panda');assert 'converged' in run()
    with page.expect_download() as download:
        page.locator('#download').click()
    assert download.value.suggested_filename.endswith('.csv')
    page.set_viewport_size({'width':390,'height':844})
    page.screenshot(path=str(ROOT/'docs/workbench-mobile.png'),full_page=True)
    assert not page.evaluate('document.documentElement.scrollWidth>innerWidth')
    assert not errors,errors
    assert not failed_assets,failed_assets
    print('PASS: playback, realistic/collision geometry, humanoid, obstacle, G1, URDF import, solve, CSV and mobile layout',flush=True)
    browser.close()
