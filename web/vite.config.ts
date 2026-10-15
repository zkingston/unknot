import { defineConfig } from 'vitest/config';
import { viteSingleFile } from 'vite-plugin-singlefile';

export default defineConfig({
  base: './',
  plugins: [viteSingleFile({ removeViteModuleLoader: true })],
  build: { target: 'es2022', assetsInlineLimit: () => true, cssCodeSplit: false, reportCompressedSize: true },
  worker: { format: 'iife' },
  test: { include: ['test/unit/**/*.test.ts'], testTimeout: 60_000 },
});
