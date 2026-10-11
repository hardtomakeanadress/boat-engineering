#!/bin/bash
# Page sanity: fail on ANY browser console error (catches JS syntax/logic
# breaks in PAGE - the whole app dies silently on those). Usage:
#   scripts/page_check.sh [port]
P=${1:-8765}
OUT=$(google-chrome --headless=new --disable-gpu --enable-logging=stderr -v=0 \
      --virtual-time-budget=9000 --dump-dom "http://127.0.0.1:$P/" 2>&1 >/dev/null)
if echo "$OUT" | grep -qiE "INFO:CONSOLE.*(uncaught|syntaxerror|typeerror|referenceerror)"; then
    echo "PAGE CHECK FAILED:"
    echo "$OUT" | grep -iE "INFO:CONSOLE" | head -5
    exit 1
fi
DOM=$(google-chrome --headless=new --disable-gpu --virtual-time-budget=9000 \
      --dump-dom "http://127.0.0.1:$P/" 2>/dev/null)
if ! echo "$DOM" | grep -q "Hull speed:" ; then
    echo "PAGE CHECK FAILED: stats never rendered (update() did not complete)"
    exit 1
fi
echo "page check: PASS (no console errors, stats rendered)"
