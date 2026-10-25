// Untrusted controller host. The parent grants only this file and the artifact read access.
import { readFileSync, readSync, writeSync } from 'node:fs';

const FORBIDDEN = /import|require|process|globalThis|constructor|prototype|__proto__|eval|Function|Reflect|Proxy|fetch|Date|performance/;
const source = readFileSync(process.argv[2], 'utf8');
if (FORBIDDEN.test(source)) throw new Error('forbidden source token');
const module = await import('data:text/javascript;base64,' + Buffer.from(source).toString('base64'));
if (typeof module.default !== 'function') throw new Error('expected default controller factory');
const controller = module.default();
if (!controller || typeof controller.step !== 'function') throw new Error('expected step(obs)');
// Raw stdio also works where creating Node net.Socket wrappers for child pipes is denied.
const send = (message) => writeSync(1, JSON.stringify(message) + '\n');
send({ ready: true });
const chunk = Buffer.alloc(8192);
let buffer = '', done = false;
while (!done) {
  const count = readSync(0, chunk, 0, chunk.length, null);
  if (!count) break;
  buffer += chunk.toString('utf8', 0, count);
  let end;
  while ((end = buffer.indexOf('\n')) >= 0) {
    const line = buffer.slice(0, end); buffer = buffer.slice(end + 1);
  try {
    const request = JSON.parse(line);
    const action = controller.step(request.obs);
    send({ action });
  } catch (error) {
    send({ error: String(error.message).slice(0, 180) });
    done = true;
    break;
  }
  }
}
