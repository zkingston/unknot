import { defineConfig, devices } from '@playwright/test';
import { existsSync } from 'node:fs';

// Local-only: on distros Playwright doesn't support, WebKit's missing libs are symlinked into its sys/lib.
if (existsSync(`${process.env.HOME}/.cache/ms-playwright/webkit-deps`)) process.env.PLAYWRIGHT_SKIP_VALIDATE_HOST_REQUIREMENTS = '1';

export default defineConfig({
  testDir: 'test/e2e',
  timeout: 60_000,
  fullyParallel: true,
  reporter: 'list',
  use: { baseURL: 'http://localhost:4173/' },
  projects: [
    { name: 'chromium', use: { ...devices['Desktop Chrome'] } },
    { name: 'firefox', use: { ...devices['Desktop Firefox'] } },
    { name: 'webkit', use: { ...devices['Desktop Safari'] } },
  ],
  // build first, so the bundle and file:// checks never run against a stale dist
  webServer: { command: 'npm run build && npm run preview', url: 'http://localhost:4173/', reuseExistingServer: true },
});
