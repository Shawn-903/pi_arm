#!/usr/bin/env python3
"""Browser acceptance against the isolated commissioning/vcan0 task fixture."""

import argparse

from playwright.sync_api import sync_playwright


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8001/control.html?ws=ws://127.0.0.1:8766')
    args = parser.parse_args()
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(headless=True)
        page = browser.new_page()
        errors = []
        page.on('pageerror', lambda error: errors.append(str(error)))
        page.goto(args.url)
        page.locator('#connectBtn').click()
        page.wait_for_function('lastState?.commissioning && lastState?.motor_test_permitted')
        assert page.evaluate('lastState.can_interface') == 'vcan0', 'virtual CAN only'
        before = page.evaluate('({id: lastState.software_zero_id, raw: lastState.raw_protocol_positions_deg})')
        page.locator('#viewRole').select_option('developer')
        page.locator('[data-view-target="commissioning"]').click()
        button = page.locator('#setZeroReference')
        assert button.is_disabled()
        page.locator('#zeroReferenceConfirmed').check()
        page.wait_for_function('!document.getElementById("setZeroReference").disabled')
        button.click()
        page.wait_for_function('document.getElementById("zeroReferenceResult").textContent.includes("软件标零成功")')
        page.wait_for_function('id => lastState.software_zero_id && lastState.software_zero_id !== id', arg=before['id'])
        assert page.evaluate('lastState.raw_protocol_positions_deg') == before['raw']
        assert max(abs(value) for value in page.evaluate('lastState.model_output_positions_rad')) < 1e-8
        assert not page.locator('#zeroReferenceConfirmed').is_checked()
        assert button.is_disabled()
        assert not errors, errors
        browser.close()
    print('PASS web software zero: explicit confirmation, task response, persisted reference ID and unchanged raw readings')


if __name__ == '__main__':
    main()
