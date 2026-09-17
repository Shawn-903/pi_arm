#!/usr/bin/env python3
"""Interactive UI against an isolated MOCK service; refuses real hardware.

Exercises only mock motion, backend IK/LIN/output previews and gesture gates.
Requires Playwright/Chromium. Example --url http://127.0.0.1:8001/control.html?ws=ws://127.0.0.1:8766
"""
import argparse
import json
import math
from pathlib import Path
from playwright.sync_api import sync_playwright
from test_web_viewer import forward


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', required=True)
    parser.add_argument('--screenshots', type=Path)
    args = parser.parse_args()
    with sync_playwright() as pw:
        browser = pw.chromium.launch(headless=True, args=['--enable-unsafe-swiftshader'])
        page = browser.new_page(viewport=dict(width=1440, height=1100))
        errors, sent = [], []
        page.on('pageerror', lambda e: errors.append(str(e)))
        page.on('websocket', lambda ws: ws.on('framesent', lambda raw: sent.append(json.loads(raw))))
        page.goto(args.url)
        page.locator('#connectBtn').click()
        page.wait_for_function('lastState?.state_available && PiArmViewer.snapshot().valid', timeout=120000)
        assert page.evaluate('lastState.hardware_type') == 'mock', 'REFUSE REAL HARDWARE'
        page.wait_for_function('lastState.preview_modes?.length === 4')
        original_task = page.evaluate('lastState.task.task_id')
        def motion_count():
            return sum(m['cmd'] not in ('preview_motion', 'enable_motor', 'disable_motor', 'stop_motion') for m in sent)
        def task_done(previous):
            page.wait_for_function('(n)=>lastState.task.task_id>n && lastState.task.status===2', arg=previous, timeout=60000)
            page.wait_for_function('!document.getElementById("releaseExecute").disabled')
        def preview(button):
            # Software WebGL on a Pi may miss the real 1.5 s freshness budget.
            # Keep the production fail-closed gate; retry only that explicit case.
            for attempt in range(3):
                page.locator(button).click()
                page.wait_for_function('!document.getElementById("planningStatus").textContent.includes("正在计算")', timeout=20000)
                text = page.locator('#planningStatus').inner_text()
                if '反馈不可用或机械臂正在运动' in text and attempt < 2:
                    print('RETRY preview after fail-closed freshness invalidation', flush=True)
                    page.wait_for_function('PiArmViewer.snapshot().valid && lastState.task.status!==1')
                    continue
                assert '失败' not in text, text
                assert page.locator('#previewJointResult span').count() == 6, text
                return
        # Preview while motors disabled proves no enable/execute side effect.
        page.locator('[data-tab="outputControlCard"]').click()
        page.locator('#copyOutputTargets').click()
        page.locator('#outputTarget1').fill('2')
        preview('#previewOutput')
        snap = page.evaluate('PiArmViewer.snapshot()')
        assert math.isclose(snap['previewJoints']['joint_01'][0], math.radians(2*65/79), abs_tol=1e-8)
        assert snap['pathPointCount'] == 201
        assert motion_count() == 0 and page.evaluate('lastState.task.task_id') == original_task
        page.locator('#pathScrub').fill('0')
        assert math.isclose(page.evaluate('PiArmViewer.snapshot().previewJoints.joint_01[0]'), 0, abs_tol=1e-6)
        page.locator('#pathScrub').fill('200')
        page.locator('#outputTarget1').fill('10000')
        assert not page.evaluate('PiArmViewer.snapshot().previewVisible')
        page.locator('#previewOutput').click()
        page.wait_for_function('document.getElementById("planningStatus").textContent.includes("失败")')
        assert page.evaluate('PiArmViewer.snapshot().pathPointCount') == 0
        print('PASS output inverse, sampled path/scrub, limits, old preview invalidation; no motion or enable', flush=True)

        page.locator('[data-cmd="enable_motor"][data-motor-id="0"]').click()
        page.wait_for_function('lastState.robot_state==="READY"')
        page.locator('[data-tab="single"]').click()
        slider = page.locator('#jointSlider1')
        slider.focus(); slider.press('ArrowRight')
        assert motion_count() == 0
        assert page.evaluate('PiArmViewer.snapshot().previewVisible')
        page.locator('#releaseExecute').check()
        old = page.evaluate('lastState.task.task_id')
        slider.focus(); slider.press('ArrowRight')
        task_done(old)
        assert motion_count() == 1
        assert abs(page.evaluate('lastState.current_joint_angles[0]') - .2) < .01
        print('PASS single-axis slider preview and one opt-in release command', flush=True)

        # Escape and pointercancel must not send, even when release execution is enabled.
        page.locator('#releaseExecute').check()
        slider.focus(); page.keyboard.down('ArrowRight'); page.keyboard.press('Escape'); page.keyboard.up('ArrowRight')
        assert motion_count() == 1 and not page.locator('#releaseExecute').is_checked()
        page.locator('#releaseExecute').check()
        box = slider.bounding_box()
        page.mouse.move(box['x'] + box['width'] / 2, box['y'] + box['height'] / 2)
        page.mouse.down(); page.mouse.move(box['x'] + box['width'] / 2 + 3, box['y'] + box['height'] / 2)
        slider.dispatch_event('pointercancel', dict(pointerId=1))
        page.mouse.up()
        assert motion_count() == 1
        page.evaluate('window.dispatchEvent(new Event("blur"))')
        assert not page.locator('#releaseExecute').is_checked()
        print('PASS Escape, pointercancel and loss-of-focus cancel without commands', flush=True)

        # Explicit selection + model-area horizontal joint gesture (not camera orbit).
        page.locator('#viewerJoints button').nth(0).click()
        page.locator('#modelInteraction').select_option('joint')
        canvas = page.locator('#viewerCanvas canvas'); canvas.scroll_into_view_if_needed()
        box = canvas.bounding_box()
        x,y = box['x']+20, box['y']+box['height']/2
        page.mouse.move(x,y); page.mouse.down(); page.mouse.move(x+5,y,steps=3); page.mouse.up()
        assert motion_count() == 1
        assert math.isclose(page.evaluate('PiArmViewer.snapshot().previewJoints.joint_01[0]'), math.radians(1.2), abs_tol=.001)
        page.locator('#releaseExecute').check()
        old = page.evaluate('lastState.task.task_id')
        canvas.scroll_into_view_if_needed(); box = canvas.bounding_box(); x,y=box['x']+20,box['y']+box['height']/2
        page.mouse.move(x,y); page.mouse.down(); page.mouse.move(x+5,y,steps=3); page.mouse.up()
        task_done(old)
        assert motion_count() == 2
        assert abs(page.evaluate('lastState.current_joint_angles[0]') - 1.2) < .02
        page.locator('#releaseExecute').uncheck()
        page.locator('#modelInteraction').select_option('orbit')
        print('PASS model drag preview and exactly one gated finite mock command', flush=True)

        # A non-singular mock anchor supports a meaningful multi-joint LIN plan.
        page.locator('[data-tab="multi"]').click()
        anchor = [-30,15,-20,-10,10,30]
        for i,q in enumerate(anchor,1): page.locator(f'#multiJoint{i}').fill(str(q))
        page.locator('#multiSpeed').fill('5')
        page.locator('#outputSpeedLimit').fill('60')  # MOCK ONLY; never applies to a real service.
        preview('#previewJoint')
        end = page.evaluate('PiArmViewer.snapshot().previewJoints')
        for name,q in zip(page.evaluate('PiArmViewer.snapshot().jointNames'),anchor):
            assert math.isclose(end[name][0], math.radians(q), abs_tol=1e-8)
        old = page.evaluate('lastState.task.task_id')
        page.locator('#multiMoveBtn').click(); task_done(old)
        assert page.evaluate('PiArmViewer.snapshot().pathPointCount') == 0
        page.locator('#outputSpeedLimit').fill('1')
        page.locator('[data-tab="cartesian"]').click()
        for name,q in zip(['X','Y','Z','Rx','Ry','Rz'],[100,-200,950,45,45,-45]):
            page.locator(f'#target{name}').fill(str(q))
        preview('#previewIK')
        snap = page.evaluate('PiArmViewer.snapshot()')
        angles = [math.degrees(snap['previewJoints'][n][0]) for n in snap['jointNames']]
        tcp = forward(angles)['tool0'][:3,3]*1000
        assert max(abs(tcp-[100,-200,950])) < .5, tcp
        page.locator('#endSpeed').fill('10'); page.locator('#lineAccel').fill('2'); page.locator('#angularSpeed').fill('1')
        before = motion_count()
        preview('#previewLIN')
        assert page.evaluate('PiArmViewer.snapshot().pathPointCount') == 201
        assert motion_count() == before
        page.locator('#playPreview').click(); page.wait_for_timeout(200)
        assert float(page.locator('#pathScrub').input_value()) < 200
        page.locator('#playPreview').click()
        print('PASS IK target FK, multi-joint LIN planning/path/playback; preview sends no motion', flush=True)
        if args.screenshots:
            args.screenshots.mkdir(parents=True,exist_ok=True)
            page.locator('#pathScrub').fill('200')
            page.locator('#robotViewer').screenshot(path=str(args.screenshots/'interactive-desktop.png'), style='.topbar {position: static !important}')
            page.set_viewport_size(dict(width=390,height=844))
            assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
            page.locator('#robotViewer').screenshot(path=str(args.screenshots/'interactive-mobile.png'), style='.topbar {position: static !important}')
            page.set_viewport_size(dict(width=1440,height=1100))
        page.locator('#targetX').fill('100000')
        page.locator('#previewIK').click()
        page.wait_for_function('document.getElementById("planningStatus").textContent.includes("失败")',timeout=20000)
        assert not page.evaluate('PiArmViewer.snapshot().previewVisible')
        assert page.evaluate('PiArmViewer.snapshot().pathPointCount') == 0
        page.locator('#releaseExecute').check()
        page.locator('#disconnectBtn').click()
        assert not page.locator('#releaseExecute').is_checked()
        assert page.locator('#jointSlider1').is_disabled()
        assert not errors, errors
        print('PASS unreachable IK hides old path, responsive layout, disconnect disarms; zero JS errors',flush=True)
        browser.close()


if __name__=='__main__': main()
