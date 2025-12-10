import { spawn } from "child_process";

console.log(JSON.stringify({
  mcp: "1.0",
  capabilities: { run: true }
}));

process.stdin.on("data", () => {
  const child = spawn("npm", ["run", "start"], { stdio: "inherit", shell: true });
  child.on("close", () => process.exit(0));
});
