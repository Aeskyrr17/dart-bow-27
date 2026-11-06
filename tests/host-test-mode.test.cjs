const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const hostRoot = process.env.HOSTCOMM_DIR || path.resolve(__dirname, '../../../DartHostComm');
const { TYPES, TEST_ACTION } = require(path.join(hostRoot, 'src/protocol/constants'));
const builders = require(path.join(hostRoot, 'src/protocol/payload-builders'));
const { decodeTestState, decodeFramePayload } = require(path.join(hostRoot, 'src/protocol/payload-decoders'));
const { buildFrame } = require(path.join(hostRoot, 'src/protocol/frame-builder'));
const { FrameParser } = require(path.join(hostRoot, 'src/protocol/frame-parser'));

function testPayload() {
  const payload = Buffer.alloc(23);
  payload.set([1, 1, TEST_ACTION.TENSION, 2, 6, 2]);
  payload.writeFloatLE(32, 6);
  payload.writeFloatLE(35, 10);
  payload.set([2, 1, 1, 1, 1], 14);
  payload.writeFloatLE(0.01, 19);
  return payload;
}

function renderer(api = {}) {
  const html = fs.readFileSync(path.join(hostRoot, 'index.html'), 'utf8');
  const elements = new Map([...html.matchAll(/id="([^"]+)"/g)].map((match) => [match[1], {
    textContent: '', value: '', disabled: false, children: [],
    appendChild(child) { this.children.push(child); },
    classList: { add() {}, remove() {}, toggle() {} },
  }]));
  elements.get('testTensionLeft').value = '32';
  elements.get('testTensionRight').value = '35';
  const context = vm.createContext({
    window: { hostComm: api },
    document: {
      getElementById(id) {
        assert.ok(elements.has(id), `missing DOM element ${id}`);
        return elements.get(id);
      },
      createElement: () => ({ textContent: '', className: '' }),
    },
    Date, Number, Set, Map, Boolean, String, performance: { now: () => 1000 },
  });
  const source = fs.readFileSync(path.join(hostRoot, 'src/renderer/renderer.js'), 'utf8')
    .replace(/init\(\)\.catch\([^\n]+\);/, '');
  vm.runInContext(source + '\nthis.controls = { state, applyTestState, renderTestControls, startTestTension, setTestAim, updateConnectionUi, applyForceTelemetry };', context);
  context.controls.state.connected = true;
  return { ...context.controls, elements };
}

test('new requests encode aim switch and independent tension without issuing fire', () => {
  assert.equal(TYPES.SET_TEST_AIM, 0x1a);
  assert.deepEqual([...builders.buildSetTestAim(true)], [1]);
  assert.deepEqual([...builders.buildSetTestAim(false)], [0]);
  assert.deepEqual([...builders.buildTestAction(TEST_ACTION.TENSION)], [4]);
  const tension = builders.buildSetTestTension(32, 35);
  assert.equal(tension.length, 8);
  assert.equal(tension.readFloatLE(0), 32);
  assert.equal(tension.readFloatLE(4), 35);
  for (const value of [-1, 121, NaN, Infinity]) {
    assert.throws(() => builders.buildSetTestTension(value, 32));
    assert.throws(() => builders.buildSetTestTension(32, value));
  }
  for (const value of [0, 5, 1.5, NaN]) assert.throws(() => builders.buildTestAction(value));
});

test('extended TEST_STATE fields decode at firmware offsets', () => {
  const state = decodeTestState(testPayload());
  assert.equal(state.extended, true);
  assert.equal(state.tensionLeftKg, 32);
  assert.equal(state.tensionRightKg, 35);
  assert.equal(state.aimState, 2);
  assert.equal(state.visionValid, true);
  assert.equal(state.tensionEnabled, true);
  assert.equal(state.tensionLeftReady, true);
  assert.equal(state.tensionRightReady, true);
  assert.ok(Math.abs(state.visionYawError - 0.01) < 1e-6);
});

test('legacy TEST_STATE remains readable and malformed lengths are rejected', () => {
  const state = decodeTestState(testPayload().subarray(0, 14));
  assert.equal(state.extended, false);
  assert.equal(state.tensionRightKg, 35);
  assert.equal(state.visionValid, false);
  for (const size of [0, 13, 15, 22, 24]) assert.throws(() => decodeTestState(Buffer.alloc(size)));
});

test('new state survives fragmented serial frame and CRC corruption is rejected', () => {
  const frame = buildFrame(TYPES.TEST_STATE, 17, testPayload());
  const parser = new FrameParser();
  assert.equal(parser.push(frame.subarray(0, 9)).length, 0);
  const [parsed] = parser.push(frame.subarray(9));
  assert.equal(parsed.seq, 17);
  assert.equal(decodeFramePayload(parsed).aimState, 2);
  const damaged = Buffer.from(frame);
  damaged[7] ^= 1;
  assert.equal(new FrameParser().push(damaged).length, 0);
});

test('manual fire requires READY and both tensions ready; aim stays separate', () => {
  const ui = renderer();
  const state = decodeTestState(testPayload());
  ui.applyTestState(state);
  assert.equal(ui.elements.get('testFireBtn').disabled, false);
  assert.equal(ui.elements.get('testAimBtn').disabled, false);
  assert.equal(ui.elements.get('testAimState').textContent, '已对准 / 保持');
  ui.applyTestState({ ...state, tensionRightReady: false });
  assert.equal(ui.elements.get('testFireBtn').disabled, true);
  ui.applyTestState({ ...state, launcherState: 6 });
  assert.equal(ui.elements.get('testFireBtn').disabled, true);
  assert.equal(ui.elements.get('testTensionBtn').disabled, false);
  ui.applyTestState({ ...state, aimState: 1 });
  assert.equal(ui.elements.get('stopTestAimBtn').disabled, false);
  assert.equal(ui.elements.get('testAimBtn').disabled, true);
});

test('invalid vision, pending fire, errors, stale state and disconnect disable controls', () => {
  const ui = renderer();
  const state = decodeTestState(testPayload());
  ui.applyTestState({ ...state, visionValid: false });
  assert.equal(ui.elements.get('testAimBtn').disabled, true);
  ui.applyTestState({ ...state, testAction: TEST_ACTION.FIRE });
  for (const id of ['testFireBtn', 'testAimBtn', 'testTensionBtn', 'testPrepareBtn']) {
    assert.equal(ui.elements.get(id).disabled, true, id);
  }
  ui.applyTestState({ ...state, launcherState: 5 });
  assert.equal(ui.elements.get('testTensionBtn').disabled, true);
  ui.applyTestState(state);
  ui.state.testStateAt = Date.now() - 1500;
  ui.renderTestControls();
  assert.equal(ui.elements.get('testFireBtn').disabled, true);
  ui.updateConnectionUi({ connected: false });
  assert.equal(ui.state.testState, null);
  assert.equal(ui.elements.get('testAimBtn').disabled, true);
});

test('old firmware disables new test features and bad packets clear prior state', () => {
  const ui = renderer();
  ui.applyTestState(decodeTestState(testPayload().subarray(0, 14)));
  assert.equal(ui.elements.get('testAimBtn').disabled, true);
  assert.equal(ui.elements.get('testTensionBtn').disabled, true);
  ui.applyTestState({ decodeError: 'invalid length' });
  assert.equal(ui.state.testState, null);
  assert.equal(ui.elements.get('testFireBtn').disabled, true);
});

test('tension button first applies both inputs then starts holding', async () => {
  const calls = [];
  const ui = renderer({
    setTestTension: async (...args) => calls.push(['tension', ...args]),
    testAction: async (action) => { calls.push(['action', action]); return { ack: { resultName: 'ACCEPTED' } }; },
  });
  ui.applyTestState(decodeTestState(testPayload()));
  await ui.startTestTension();
  assert.deepEqual(calls, [['tension', 32, 35], ['action', TEST_ACTION.TENSION]]);
});

test('rejected tension update never starts tension or fire', async () => {
  const calls = [];
  const ui = renderer({
    setTestTension: async () => { throw new Error('PARAM_OUT_OF_RANGE'); },
    testAction: async (action) => calls.push(action),
  });
  await ui.startTestTension();
  assert.deepEqual(calls, []);
});

test('aim buttons only send aim commands; feedback appears independently of chart', async () => {
  const calls = [];
  const ui = renderer({ setTestAim: async (enabled) => {
    calls.push(enabled); return { ack: { resultName: 'ACCEPTED' } };
  } });
  await ui.setTestAim(true);
  await ui.setTestAim(false);
  assert.deepEqual(calls, [true, false]);
  ui.applyForceTelemetry({ feedbackLeftKg: 31.99, feedbackRightKg: 35.01 });
  assert.equal(ui.elements.get('testForceLeft').textContent, '31.99 kg');
  assert.equal(ui.elements.get('testForceRight').textContent, '35.01 kg');
});
