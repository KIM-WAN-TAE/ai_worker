import json
from pathlib import Path
from urllib.request import urlopen

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).parent
URL = 'http://127.0.0.1:18767'


def state(arm='left'):
    with urlopen(f'{URL}/api/state?arm={arm}') as response:
        return json.load(response)


with sync_playwright() as playwright:
    browser = playwright.chromium.launch(headless=True, args=['--no-sandbox'])
    page = browser.new_page(viewport={'width': 1600, 'height': 1100})
    page.add_init_script('''
        window.requestAnimationFrame = callback => setTimeout(() => callback(performance.now()), 200);
        window.cancelAnimationFrame = clearTimeout;
    ''')
    errors = []
    page.on('pageerror', lambda error: errors.append(str(error)))
    page.goto(URL + '/generator')
    print('Generator page loaded', flush=True)
    with urlopen(URL + '/api/waypoints/export?arm=left') as response:
        reference = response.read()
    page.locator('#referenceFile').set_input_files({
        'name': 'left_reference.json', 'mimeType': 'application/json', 'buffer': reference,
    })
    page.wait_for_function("!document.getElementById('referenceCard').classList.contains('disabled')")
    page.locator('#graspCompleteWaypoint').fill('2')
    page.locator('#graspConfirmed').check()
    page.locator('#offsetX').fill('0.5')
    page.locator('#offsetY').fill('-0.5')
    page.locator('#offsetZ').fill('0.5')
    page.locator('#generateButton').click()
    print('Generator submitted', flush=True)
    page.locator('#resultCard').wait_for(state='visible', timeout=90000)
    generator_result = page.locator('#resultMessage').inner_text()
    print(generator_result, flush=True)
    assert page.locator('#downloadButton').is_enabled()
    page.screenshot(path=str(ROOT / 'generator.png'), full_page=True, timeout=60000)
    print('Generator screenshot saved', flush=True)
    page.goto(URL)
    print('Waypoint page loaded', flush=True)
    page.locator('#refinementEnabled').check()
    page.locator('#refinementSubdivisions').fill('7')
    page.locator('#refinementStrengthNumber').fill('60')
    assert page.locator('#refinementStrength').input_value() == '60'
    with page.expect_response('**/api/path-refinement') as applied:
        page.locator('#applyRefinement').click()
    assert applied.value.ok, applied.value.text()
    page.wait_for_function("document.getElementById('refinementMonitor').dataset.stage === 'success'", timeout=90000)
    left = state()
    assert left['path_refinement']['subdivisions'] == 8
    assert left['path_refinement']['smoothing_percent'] == 60
    assert page.locator('#equalTimeAll').is_enabled()
    page.locator('#equalTimeMode').select_option('segment')
    page.locator('#equalTimeSeconds').fill('1.5')
    with page.expect_response('**/api/waypoints/equal-time', timeout=90000) as equal:
        page.locator('#equalTimeAll').click()
    assert equal.value.ok, equal.value.text()
    page.wait_for_function("!document.getElementById('equalTimeAll').disabled")
    page.wait_for_function("document.getElementById('equalTimeResult').textContent.startsWith('시간 배정 검증 완료')")
    segment_result = page.locator('#equalTimeResult').inner_text()
    left = state()
    arrivals = [w['trajectory_time_s'] for w in left['waypoints']]
    assert all(abs(arrivals[i] - arrivals[i - 1] - 1.5) < 1e-6 for i in range(1, 4)), arrivals
    page.locator('#equalTimeMode').select_option('total')
    page.locator('#equalTimeSeconds').fill('6')
    with page.expect_response('**/api/waypoints/equal-time', timeout=90000) as equal:
        page.locator('#equalTimeAll').click()
    assert equal.value.ok, equal.value.text()
    page.wait_for_function("!document.getElementById('equalTimeAll').disabled")
    page.wait_for_function("document.getElementById('equalTimeResult').textContent.startsWith('전체 목표시간 검증 완료')")
    total_result = page.locator('#equalTimeResult').inner_text()
    left = state()
    arrivals = [w['trajectory_time_s'] for w in left['waypoints']]
    assert abs(arrivals[-1] - arrivals[0] - 6) < 1e-6, arrivals
    page.screenshot(path=str(ROOT / 'refinement_equal_time.png'), full_page=True)
    page.locator('[data-arm="right"]').first.click()
    page.wait_for_function("!document.getElementById('refinementEnabled').checked")
    assert not state('right')['path_refinement']['enabled']
    page.locator('[data-arm="left"]').first.click()
    page.wait_for_function("document.getElementById('refinementEnabled').checked")
    assert page.locator('#refinementSubdivisions').input_value() == '7'
    assert page.locator('#refinementStrengthNumber').input_value() == '60'
    assert not errors, errors
    report = {
        'passed': True,
        'generator': generator_result,
        'equal_segment': segment_result,
        'equal_total': total_result,
        'arrival_times': arrivals,
        'refinement': left['path_refinement'],
        'diagnostics': left.get('path_refinement_events'),
        'javascript_errors': errors,
        'arm_settings_isolated': True,
    }
    (ROOT / 'browser-results.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False, indent=2))
    browser.close()
