import type { EngineInterface, Register } from 'claude-code'

// /bgt: /bg (the built-in /background), after writing this session's id to the file the `claude` shell function in
// bash_aliases names in CLAUDE_BGT_NOTE. Once claude exits, that function finds the file and runs `cbg` on the
// session: `claude respawn` and `claude attach` in tmux.

// A command.run hook can't run another command itself (it would wait on the turn the hook holds): it answers at once
// and runs the built-in a moment later, on a timer of its own
function later($: EngineInterface, run: () => Promise<unknown>): void {
  void $.clock.after(1, () => {
    void run().catch(e => $.ui.toast(`bg-tmux: ${e instanceof Error ? e.message : String(e)}`))
  })
}

export const register: Register = on => {
  on('session.start', async ($, e, next) => {
    await $.command.register({
      name: 'bgt',
      description: 'Send this session to the background (/bg), then respawn and attach to it in tmux',
    })
    return next(e)
  })

  on('command.run', { command: 'bgt' }, async ($, e) => {
    const note = await $.env.get('CLAUDE_BGT_NOTE')
    if (note) await $.fs.write(note, await $.session.id())
    later($, () => $.command.run({ command: 'background', args: e.args }))
    return {
      text: note
        ? 'moving to the background, then to tmux'
        : 'moving to the background (not started by the claude shell function: run `cbg` afterwards)',
    }
  })
}
