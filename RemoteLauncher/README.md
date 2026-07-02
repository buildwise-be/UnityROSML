# RemoteLauncher

Small C++ programs that run on a **remote PC** (Windows or Linux). They receive pose data from the Unity Magic Leap app over Wi-Fi and run your own executable with that data.

This folder contains two programs:

| File | What it does |
|------|----------------|
| `remote_launcher.cpp` | **Server** — stays running, listens on a TCP port, starts your task when Unity calls |
| `example_task.cpp` | **Example task** — template for your own program; reads poses from stdin |

You will usually run **both**: the launcher as a background service, and your task as the program it starts.

---

## What you need before building

### Linux (Ubuntu / Debian)

Open a terminal and install the compiler and CMake:

```bash
sudo apt update
sudo apt install build-essential cmake
```

That gives you `g++` and `cmake`.

### Windows

Install **one** of these (pick the easiest for you):

1. **Visual Studio Community** (recommended for beginners)  
   Download from [https://visualstudio.microsoft.com/](https://visualstudio.microsoft.com/)  
   During install, check **“Desktop development with C++”**.

2. **Or** install [CMake](https://cmake.org/download/) + “Build Tools for Visual Studio”.

You also need **CMake** on Windows if you use the CMake method below:  
[https://cmake.org/download/](https://cmake.org/download/)

---

## How to build (easiest method — CMake)

This works the same way on Windows and Linux.

1. Open a terminal:
   - **Linux:** normal Terminal
   - **Windows:** “x64 Native Tools Command Prompt for VS” **or** PowerShell

2. Go into this folder:

```bash
cd path/to/UnityROSML/RemoteLauncher
```

3. Build:

```bash
cmake -B build
cmake --build build --config Release
```

4. When it finishes, your programs are here:

| Platform | Location |
|----------|----------|
| **Linux** | `build/remote_launcher` and `build/example_task` |
| **Windows** | `build\Release\remote_launcher.exe` and `build\Release\example_task.exe` |

If the Windows `Release` folder is empty, also try `build\Debug\`.

---

## How to build (without CMake)

Use this if CMake gives you trouble.

### Linux

```bash
cd RemoteLauncher
g++ -std=c++17 -O2 -o remote_launcher remote_launcher.cpp
g++ -std=c++17 -O2 -o example_task example_task.cpp
```

You get two files in the same folder: `remote_launcher` and `example_task`.

### Windows

Open **“x64 Native Tools Command Prompt for VS”**, then:

```bat
cd path\to\UnityROSML\RemoteLauncher
cl /std:c++17 /O2 remote_launcher.cpp ws2_32.lib
cl /std:c++17 /O2 example_task.cpp
```

You get `remote_launcher.exe` and `example_task.exe` in the same folder.

---

## How to deploy and run

“Deploy” here means: copy the built programs to the machine that will sit on the same Wi-Fi as the Magic Leap, then start the server.

### Step 1 — Copy files to the remote machine

Copy at least:

- `remote_launcher` (or `remote_launcher.exe`)
- `example_task` (or `example_task.exe`) — or your own program later

Put them in a folder you will remember, e.g.:

- Linux: `/home/you/remote_launcher/`
- Windows: `C:\remote_launcher\`

Both `.exe` / binaries must be in the **same folder**, or you must pass the full path to your task when starting the launcher.

### Step 2 — Allow the port through the firewall

The default port is **6001** (TCP). The remote machine must accept incoming connections on that port from your Wi-Fi network.

**Linux (Ubuntu, quick test):**

```bash
sudo ufw allow 6001/tcp
```

**Windows:**

1. Open **Windows Defender Firewall** → **Advanced settings**
2. **Inbound Rules** → **New Rule**
3. Port → TCP → **6001** → Allow → give it a name (e.g. “Remote Launcher”)

### Step 3 — Start the server

Open a terminal on the remote machine, go to your folder, and run:

**Linux:**

```bash
cd /home/you/remote_launcher
./remote_launcher 6001 ./example_task
```

**Windows (Command Prompt or PowerShell):**

```bat
cd C:\remote_launcher
remote_launcher.exe 6001 example_task.exe
```

You should see something like:

```text
remote_launcher listening on port 6001, target executable: ./example_task
```

Leave this window **open**. The server runs until you close it (Ctrl+C).

**Arguments:**

```text
remote_launcher <port> <path-to-your-executable>
```

- `<port>` — usually `6001` (must match Unity’s **Server Port**)
- `<path-to-your-executable>` — the program that receives the pose JSON on stdin

### Step 4 — Point Unity at this machine

In Unity, on `RemoteExecutorClient`:

- **Server Host** → IP address of the remote PC (e.g. `192.168.1.100`)
- **Server Port** → `6001` (same as above)

To find the PC’s IP:

- **Linux:** `hostname -I` or `ip addr`
- **Windows:** `ipconfig` → look for **IPv4 Address** on your Wi-Fi adapter

Magic Leap and the remote PC must be on the **same Wi-Fi network**.

---

## Quick test (without Unity)

From another machine on the same network, you can check that the server works.

**Linux / macOS** (replace `192.168.1.100` with your server IP):

```bash
printf '{"command":"run_task","poses":[{"frame_id":"test","sec":0,"nanosec":0,"position":{"x":1,"y":2,"z":3},"orientation":{"x":0,"y":0,"z":0,"w":1}}]}\n' | nc 192.168.1.100 6001
```

**Windows PowerShell:**

```powershell
$c = New-Object Net.Sockets.TcpClient('192.168.1.100', 6001)
$s = $c.GetStream()
$json = '{"command":"run_task","poses":[{"frame_id":"test","sec":0,"nanosec":0,"position":{"x":1,"y":2,"z":3},"orientation":{"x":0,"y":0,"z":0,"w":1}}]}' + "`n"
$b = [Text.Encoding]::UTF8.GetBytes($json)
$s.Write($b, 0, $b.Length)
(New-Object IO.StreamReader($s)).ReadLine()
$c.Close()
```

Expected reply:

```json
{"ok":true,"exit_code":0}
```

On the server terminal you should see the connection and `example_task` printing the pose.

---

## Run on startup (optional)

### Linux — simple background run

```bash
nohup ./remote_launcher 6001 ./example_task > launcher.log 2>&1 &
```

### Windows — Task Scheduler

1. Open **Task Scheduler** → **Create Basic Task**
2. Trigger: **When the computer starts** (or **At log on**)
3. Action: **Start a program**
4. Program: `C:\remote_launcher\remote_launcher.exe`
5. Arguments: `6001 example_task.exe`
6. Start in: `C:\remote_launcher`

---

## Writing your own task

1. Copy `example_task.cpp` and rename it (e.g. `my_task.cpp`).
2. Edit the `TODO` section — that is where your real logic goes.
3. Rebuild it the same way as `example_task`.
4. Start the launcher with your new executable:

```bash
./remote_launcher 6001 ./my_task
```

Your program receives **one line of JSON on stdin** (poses from Unity). Exit with code **0** for success; any other code is reported as failure to Unity.

---

## Troubleshooting

| Problem | What to try |
|---------|-------------|
| Unity says “connection timed out” | Check IP/port, firewall, same Wi-Fi, server still running |
| “bind() failed on port 6001” | Port already in use; pick another port and update Unity |
| “failed to start executable” | Wrong path to task exe; use full path or run from the same folder |
| Build fails on Windows | Use “x64 Native Tools Command Prompt for VS”, not plain cmd |
| Build fails on Linux | Run `sudo apt install build-essential cmake` |

---

## Files in this folder

- `remote_launcher.cpp` — TCP server
- `example_task.cpp` — example program (replace with yours)
- `CMakeLists.txt` — build config for CMake
- `README.md` — this file

Unity scripts that talk to this server live in `Assets/Scripts/V3/RemoteExecution/`.
