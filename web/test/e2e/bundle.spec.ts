import { test, expect } from '@playwright/test';
import { readdirSync, readFileSync } from 'node:fs';
import { gzipSync } from 'node:zlib';
import { fileURLToPath } from 'node:url';
import { resolve, dirname } from 'node:path';

const dist = resolve(dirname(fileURLToPath(import.meta.url)), '../../dist');
const distUrl = 'file://' + dist + '/index.html';

test('dist holds only index.html, <= 400 KB gzipped', () => {
  expect(readdirSync(dist)).toEqual(['index.html']);
  const kb = gzipSync(readFileSync(dist + '/index.html'), { level: 9 }).length / 1024;
  console.log(`bundle: ${kb.toFixed(1)} KB gzipped`);
  expect(kb).toBeLessThanOrEqual(400);
});

for (const [name, url] of [['preview', '/'], ['file', distUrl]] as const) {
  test(`workers answer ping and fonts render (${name})`, async ({ page }) => {
    await page.goto(url);
    await expect(page.locator('body')).toHaveAttribute('data-workers', 'ok', { timeout: 10_000 });
    const fonts = await page.evaluate(async () => {
      const faces = await Promise.all(['400 13px', 'italic 400 13px', '700 13px'].map((f) => document.fonts.load(`${f} "Iosevka Term Slab"`, '§ unknot')));
      return faces.map((fs) => fs.length > 0 && fs.every((f) => f.status === 'loaded'));
    });
    expect(fonts).toEqual([true, true, true]);
    const family = await page.locator('.title').evaluate((el) => getComputedStyle(el).fontFamily);
    expect(family).toContain('Iosevka Term Slab');
  });
}
