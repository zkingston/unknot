// Minimal Node typings for tests and tools (avoids an @types/node dev dependency).
declare module 'node:fs' {
  export function readFileSync(path: string | URL): Uint8Array & { toString(enc?: string): string };
  export function readFileSync(path: string | URL, enc: 'utf8'): string;
  export function writeFileSync(path: string | URL, data: string | Uint8Array): void;
  export function existsSync(path: string | URL): boolean;
  export function readdirSync(path: string | URL): string[];
}
declare module 'node:crypto' {
  export function createHash(alg: string): { update(d: Uint8Array | string): { digest(enc: 'hex'): string } };
}
declare module 'node:url' {
  export function fileURLToPath(u: string | URL): string;
}
declare module 'node:path' {
  export function resolve(...p: string[]): string;
  export function dirname(p: string): string;
}
declare const process: { env: Record<string, string | undefined> };
declare module 'node:zlib' {
  export function inflateSync(d: Uint8Array): Uint8Array;
  export function gzipSync(d: Uint8Array, o?: { level?: number }): Uint8Array;
}
