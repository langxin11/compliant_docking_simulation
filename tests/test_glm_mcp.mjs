/** 桥接边界测试：路径限制、凭据读取、模型错误和 MCP 协议；不调用远端模型。 */
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import crypto from 'node:crypto';
import { spawnSync } from 'node:child_process';
import { checkedPath, code, complete, decryptCredential, dispatch, loadCredential, startStatus, validateProposal } from '../scripts/glm_mcp.mjs';

async function workspace(t) {
  const root = await fs.mkdtemp(path.join(os.tmpdir(), 'glm-mcp-test-'));
  t.after(() => fs.rm(root, { recursive: true, force: true }));
  return root;
}

test('普通源码及不存在的新文件允许，敏感或越界路径拒绝', async t => {
  const root = await workspace(t);
  await fs.writeFile(path.join(root, 'sample.py'), 'print(1)\n');
  assert.equal(await checkedPath(root, 'sample.py'), path.join(root, 'sample.py'));
  assert.equal(await checkedPath(root, 'src/new.py'), path.join(root, 'src/new.py'));
  for (const name of ['../outside.py', '/tmp/a.py', '.env', '.git/config', 'runs/a.json', 'results/a.md', 'archive/a.py', 'a/credentials.json', 'a/../b.py', 'a\\b.py']) {
    await assert.rejects(checkedPath(root, name));
  }
});

test('已存在文件及不存在子文件都不能沿符号链接越界', async t => {
  const root = await workspace(t);
  await fs.symlink(os.tmpdir(), path.join(root, 'linked'));
  await assert.rejects(checkedPath(root, 'linked/new.py'), /符号链接/);
});

test('兼容 ZCode AES-GCM 格式并拒绝错误密钥', () => {
  const secret = 'test-secret';
  const iv = crypto.randomBytes(12);
  const cipher = crypto.createCipheriv('aes-256-gcm', crypto.createHash('sha256').update(secret).digest(), iv);
  const data = Buffer.concat([cipher.update('fixture-token'), cipher.final()]);
  const encoded = 'enc:v1:' + [iv, cipher.getAuthTag(), data].map(value => value.toString('base64url')).join('.');
  assert.equal(decryptCredential(encoded, secret), 'fixture-token');
  assert.throws(() => decryptCredential(encoded, 'wrong'), /无法读取/);
});

test('显式环境认证无需访问本机 ZCode 登录', async () => {
  assert.deepEqual(await loadCredential({ BIGMODEL_API_KEY: ' fixture-token ' }), { key: 'fixture-token', source: 'environment' });
});

test('提案只允许指定文件，不允许重复、无效 JSON 和大输出', () => {
  const valid = { summary: '补充中文说明', edits: [{ path: 'a.py', content: '# 中文\n' }] };
  assert.deepEqual(validateProposal(JSON.stringify(valid), ['a.py']), valid);
  assert.deepEqual(validateProposal('```json\n' + JSON.stringify(valid) + '\n```', ['a.py']), valid);
  assert.throws(() => validateProposal(JSON.stringify(valid), ['b.py']), /越界/);
  assert.throws(() => validateProposal(JSON.stringify({ ...valid, edits: [valid.edits[0], valid.edits[0]] }), ['a.py', 'b.py']), /重复/);
  assert.throws(() => validateProposal('not JSON', ['a.py']), /JSON/);
  assert.throws(() => validateProposal(JSON.stringify({ summary: '大输出', edits: [{ path: 'a.py', content: 'x'.repeat(400001) }] }), ['a.py']), /过大/);
});

test('配额失败保留平台代码并移除密钥，不重试消耗额度', async () => {
  let calls = 0;
  await assert.rejects(complete('glm-5.3', [], { key: 'fixture-token', fetchImpl: async () => {
    calls++;
    return { ok: false, status: 429, json: async () => ({ error: { code: '1310', message: '限额 fixture-token' } }) };
  } }), error => error.httpStatus === 429 && error.platformCode === '1310' && !error.message.includes('fixture-token'));
  assert.equal(calls, 1);
});

test('两个指定模型可用，其他模型及截断输出明确拒绝', async () => {
  for (const model of ['glm-5.3', 'glm-5.3-flash']) {
    const result = await complete(model, [], { key: 'test', fetchImpl: async (_url, request) => {
      assert.equal(JSON.parse(request.body).model, model);
      return { ok: true, status: 200, json: async () => ({ model, choices: [{ message: { content: 'OK' }, finish_reason: 'stop' }] }) };
    } });
    assert.equal(result.model, model);
  }
  await assert.rejects(complete('other-model', [], { key: 'test' }), /模型必须/);
  await assert.rejects(complete('glm-5.3', [], { key: 'test', fetchImpl: async () => ({ ok: true, status: 200, json: async () => ({ choices: [{ message: { content: 'partial' }, finish_reason: 'length' }] }) }) }), /截断/);
});

test('MCP 初始化、工具发现、通知和未知方法有正确回复', async () => {
  const initialized = await dispatch({ id: 1, method: 'initialize', params: { protocolVersion: '2025-06-18' } }, '/tmp');
  assert.equal(initialized.result.protocolVersion, '2025-06-18');
  const listing = await dispatch({ id: 2, method: 'tools/list' }, '/tmp');
  assert.deepEqual(listing.result.tools.map(tool => tool.name), ['glm_start_status', 'glm_status', 'glm_code']);
  assert.equal(await dispatch({ method: 'notifications/initialized' }, '/tmp'), null);
  assert.equal((await dispatch({ id: 3, method: 'unknown' }, '/tmp')).error.code, -32601);
});

test('真实 stdio 进程输出纯协议 JSON，使用环境认证而不泄露密钥', async t => {
  const root = await workspace(t);
  const result = spawnSync(process.execPath, [path.resolve('scripts/glm_mcp.mjs')], {
    input: [
      { jsonrpc: '2.0', id: 1, method: 'initialize', params: { protocolVersion: '2025-03-26' } },
      { jsonrpc: '2.0', id: 2, method: 'tools/list' },
      { jsonrpc: '2.0', id: 3, method: 'tools/call', params: { name: 'glm_status', arguments: {} } },
    ].map(value => JSON.stringify(value)).join('\n') + '\n',
    encoding: 'utf8', env: { ...process.env, GLM_WORKSPACE: root, BIGMODEL_API_KEY: 'fixture-private-token' }, timeout: 10000,
  });
  assert.equal(result.status, 0, result.stderr);
  const responses = result.stdout.trim().split('\n').map(line => JSON.parse(line));
  assert.equal(responses.length, 3);
  assert.equal(JSON.parse(responses[2].result.content[0].text).credentialSource, 'environment');
  assert.equal(result.stdout.includes('fixture-private-token'), false);
});


test('编码任务只读取指定文件并保存提案，不改写源码且附带规则和输入哈希', async t => {
  const root = await workspace(t);
  await fs.writeFile(path.join(root, 'AGENTS.md'), '中文注释规范');
  await fs.writeFile(path.join(root, 'sample.py'), 'value = 1\n');
  const result = await code({ task: '补充中文注释', files: ['sample.py'], model: 'glm-5.3-flash' }, root, {
    credentialLoader: async () => ({ key: 'fixture-token' }),
    completion: async (model, messages) => {
      assert.equal(model, 'glm-5.3-flash');
      const context = JSON.parse(messages[1].content);
      assert.equal(context.files[0].content, 'value = 1\n');
      assert.equal(context.rules[0].content, '中文注释规范');
      return { text: JSON.stringify({ summary: '中文注释', edits: [{ path: 'sample.py', content: '# 数值\nvalue = 1\n' }] }), model, usage: { total_tokens: 10 } };
    },
  });
  assert.equal(await fs.readFile(path.join(root, 'sample.py'), 'utf8'), 'value = 1\n');
  const proposal = JSON.parse(await fs.readFile(result.proposalFile, 'utf8'));
  assert.equal(proposal.before['sample.py'].length, 64);
  assert.equal(proposal.edits[0].content, '# 数值\nvalue = 1\n');
  assert.equal((await fs.stat(result.proposalFile)).mode & 0o777, 0o600);
});

test('任务期间输入变化或模型返回越界编辑时，不落盘提案', async t => {
  const root = await workspace(t);
  await fs.writeFile(path.join(root, 'sample.py'), 'value = 1\n');
  await assert.rejects(code({ task: '更新', files: ['sample.py'] }, root, {
    credentialLoader: async () => ({ key: 'fixture-token' }),
    completion: async () => {
      await fs.writeFile(path.join(root, 'sample.py'), 'value = 2\n');
      return { text: JSON.stringify({ summary: '更新', edits: [{ path: 'sample.py', content: 'value = 3\n' }] }), model: 'glm-5.3' };
    },
  }), /已变化/);
  await assert.rejects(code({ task: '更新', files: ['sample.py'] }, root, {
    credentialLoader: async () => ({ key: 'fixture-token' }),
    completion: async () => ({ text: JSON.stringify({ summary: '扩大范围', edits: [{ path: 'other.py', content: '' }] }) }),
  }), /越界/);
  await assert.rejects(fs.stat(path.join(root, 'runs/glm_bridge')), { code: 'ENOENT' });
});


test('Start Plan 权益独立查询，只显示有效模型且不把发放总量当作剩余量', async () => {
  const result = await startStatus({
    credentialLoader: async () => ({ key: 'start-private-token', source: 'zcode_login' }),
    fetchImpl: async (url, options) => {
      assert.match(url, /zcode-plan\/billing\/current$/);
      assert.equal(options.headers.Authorization, 'Bearer start-private-token');
      const plan = { name: 'Start', status: 'active', starts_at: 10, ends_at: 200,
        user_plan_id: 'private-id', entitlements: [{ effective_at: 0,
          capabilities: ['model:GLM-5.3-Flash'], grant_units: 100000000, unit_type: 'token', period: 'one_time' }] };
      return { ok: true, status: 200, json: async () => ({ code: 0, data: { server_time: 100,
        plans: [plan, { ...plan, ends_at: 99 }, { ...plan, starts_at: 150 }, { ...plan, status: 'expired' }] } }) };
    },
  });
  assert.equal(result.activePlans.length, 1);
  assert.deepEqual(result.entitledModels, ['glm-5.3-flash']);
  assert.equal(result.activePlans[0].grants[0].grantedUnits, 100000000);
  assert.equal(result.remainingUnits, null);
  assert.equal(result.liveModelConnectionVerified, false);
  assert.equal(result.codingViaBridgeSupported, false);
  assert.equal(JSON.stringify(result).includes('private-id'), false);
  assert.equal(JSON.stringify(result).includes('start-private-token'), false);
});

test('Start Plan 查询失败脱敏并不重试，不落回 Coding Plan', async () => {
  let calls = 0;
  await assert.rejects(startStatus({
    credentialLoader: async () => ({ key: 'start-private-token' }),
    fetchImpl: async () => { calls++; return { ok: false, status: 401,
      json: async () => ({ code: 3000, msg: 'invalid start-private-token' }) }; },
  }), error => error.message.includes('401') && !error.message.includes('start-private-token'));
  assert.equal(calls, 1);
});

test('Start Plan 响应缺少服务器时间时不能认定有效权益', async () => {
  await assert.rejects(startStatus({
    credentialLoader: async () => ({ key: 'test' }),
    fetchImpl: async () => ({ ok: true, status: 200, json: async () => ({ code: 0, data: { plans: [] } }) }),
  }), /不能判断/);
});
