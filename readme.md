运行环境：

https://nodejs.org

下载NodeJs，要选最新的 LTS 版本，避免串口库不兼容。

然后使用以下命令启用pnpm支持：

```bash
corepack enable pnpm
```

---

首先初始化项目依赖项：

```bash
pnpm install
```

初始化后启动编译器，从TypeScript源码产生可执行的JavaScript脚本：

```shell
pnpm run build:w
```

运行编译后的脚本：

```shell
pnpm run run:ts
```

运行后，记得使用Ctrl+C停止脚本来释放串口。

代码参见 `src/index.ts` ，最后一段是发送数据包的代码。

---


