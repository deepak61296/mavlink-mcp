/**
 * mavlink-mcp bridge for the pi coding agent.
 *
 * pi deliberately ships no MCP client ("build an extension that adds MCP support"), so this
 * is that extension, scoped to one server. It spawns mavlink-mcp over stdio, speaks the
 * newline-delimited JSON-RPC that the MCP stdio transport uses, and registers every tool the
 * server advertises as a pi tool - schema included, so pi validates arguments the same way
 * Claude and Codex do.
 *
 *   pi -e integrations/pi/mavlink-mcp.ts "take off to 20 m, orbit 15 m, then RTL"
 *
 * Configure with MAVLINK_MCP_COMMAND (default "mavlink-mcp") and MAVLINK_MCP_ARGS
 * (space separated, e.g. "--enable-actuation --camera gazebo").
 */
import { spawn, spawnSync, type ChildProcessWithoutNullStreams } from "node:child_process";
import type { ExtensionAPI } from "@earendil-works/pi-coding-agent";

const COMMAND = process.env.MAVLINK_MCP_COMMAND ?? "mavlink-mcp";
const ARGS = (process.env.MAVLINK_MCP_ARGS ?? "").split(/\s+/).filter(Boolean);
const PROTOCOL_VERSION = "2025-06-18";

interface Pending {
	resolve: (value: any) => void;
	reject: (reason: Error) => void;
}

class McpStdioClient {
	private child?: ChildProcessWithoutNullStreams;
	private pending = new Map<number, Pending>();
	private buffer = "";
	private nextId = 1;

	async start(): Promise<void> {
		const child = spawn(COMMAND, ARGS, { stdio: ["pipe", "pipe", "pipe"] });
		this.child = child;
		child.stdout.setEncoding("utf8");
		child.stdout.on("data", (chunk: string) => this.onData(chunk));
		// The server logs to stderr; surfacing it here would interleave with pi's own UI.
		child.stderr.resume();
		child.on("exit", () => {
			for (const { reject } of this.pending.values()) {
				reject(new Error("mavlink-mcp exited"));
			}
			this.pending.clear();
		});

		await this.request("initialize", {
			protocolVersion: PROTOCOL_VERSION,
			capabilities: {},
			clientInfo: { name: "pi", version: "1" },
		});
		this.notify("notifications/initialized");
	}

	private onData(chunk: string): void {
		this.buffer += chunk;
		let index: number;
		while ((index = this.buffer.indexOf("\n")) >= 0) {
			const line = this.buffer.slice(0, index).trim();
			this.buffer = this.buffer.slice(index + 1);
			if (!line) continue;
			let message: any;
			try {
				message = JSON.parse(line);
			} catch {
				continue; // not ours; the transport is line-delimited JSON only
			}
			const waiter = this.pending.get(message.id);
			if (!waiter) continue;
			this.pending.delete(message.id);
			if (message.error) waiter.reject(new Error(message.error.message ?? "mcp error"));
			else waiter.resolve(message.result);
		}
	}

	private send(payload: Record<string, unknown>): void {
		if (!this.child) throw new Error("mavlink-mcp is not running");
		this.child.stdin.write(`${JSON.stringify(payload)}\n`);
	}

	notify(method: string, params: Record<string, unknown> = {}): void {
		this.send({ jsonrpc: "2.0", method, params });
	}

	request(method: string, params: Record<string, unknown> = {}, timeoutMs = 600_000): Promise<any> {
		const id = this.nextId++;
		return new Promise((resolve, reject) => {
			// Flight commands block until the vehicle arrives, so the default ceiling is
			// generous: an RTL from altitude legitimately takes minutes.
			const timer = setTimeout(() => {
				this.pending.delete(id);
				reject(new Error(`${method} timed out`));
			}, timeoutMs);
			this.pending.set(id, {
				resolve: (value) => { clearTimeout(timer); resolve(value); },
				reject: (error) => { clearTimeout(timer); reject(error); },
			});
			this.send({ jsonrpc: "2.0", id, method, params });
		});
	}

	stop(): void {
		this.child?.stdin.end();
		this.child?.kill();
		this.child = undefined;
	}
}

/**
 * Ask the server what it offers, synchronously, in a throwaway process.
 *
 * Tools have to be registered while the extension loads: pi does not wait for an async
 * session_start handler, so discovering them in one would register nothing. This costs a
 * process start and no vehicle traffic, because the server only opens the MAVLink link on
 * the first tool call.
 */
function discoverTools(): any[] {
	const handshake = [
		{
			jsonrpc: "2.0", id: 1, method: "initialize",
			params: {
				protocolVersion: PROTOCOL_VERSION,
				capabilities: {},
				clientInfo: { name: "pi", version: "1" },
			},
		},
		{ jsonrpc: "2.0", method: "notifications/initialized" },
		{ jsonrpc: "2.0", id: 2, method: "tools/list" },
	];
	const probe = spawnSync(COMMAND, ARGS, {
		input: handshake.map((m) => JSON.stringify(m)).join("\n") + "\n",
		encoding: "utf8",
		timeout: 30_000,
	});
	for (const line of (probe.stdout ?? "").split("\n")) {
		if (!line.trim()) continue;
		try {
			const message = JSON.parse(line);
			if (message.id === 2) return message.result?.tools ?? [];
		} catch {
			// the server logs progress lines that are not JSON; skip them
		}
	}
	return [];
}

export default function mavlinkMcpExtension(pi: ExtensionAPI) {
	const client = new McpStdioClient();
	let started: Promise<void> | undefined;
	const ready = () => (started ??= client.start());

	// Discovery is synchronous and registration happens in a synchronous session_start:
	// pi does not await async lifecycle handlers, so anything registered from one arrives
	// after the model has already been given its tool list.
	let tools: any[] = [];

	const register = (tool: any) => {
		pi.registerTool({
			name: tool.name,
			label: tool.name,
			description: tool.description ?? "",
			// The server's schema passes through untouched: the bounds and enums it
			// advertises are the whole point, and rewriting them would lose them.
			parameters: tool.inputSchema,
			async execute(_id: string, params: Record<string, unknown>) {
				await ready();
				const result = await client.request("tools/call", {
					name: tool.name,
					arguments: params ?? {},
				});
				const content = (result?.content ?? []).map((block: any) =>
					block.type === "image"
						? { type: "image", data: block.data, mimeType: block.mimeType ?? "image/jpeg" }
						: { type: "text", text: block.text ?? "" },
				);
				return {
					content: content.length ? content : [{ type: "text", text: "(no output)" }],
					isError: Boolean(result?.isError),
					details: { server: "mavlink-mcp", tool: tool.name },
				};
			},
		});
	};

	pi.on("session_start", (_event, ctx) => {
		tools = discoverTools();
		for (const tool of tools) register(tool);
		if (tools.length) ctx.ui.notify(`mavlink-mcp: ${tools.length} tools`, "info");
		else ctx.ui.notify(`mavlink-mcp: no tools (is ${COMMAND} installed?)`, "error");
	});

	pi.on("session_end", () => client.stop());
}
