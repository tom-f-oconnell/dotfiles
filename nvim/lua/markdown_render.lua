-- render-markdown.nvim setup: Neovim's counterpart to the markdown_render autocmds in
-- vimrc. Markdown is shown rendered (incl. on the cursor line) except in insert / replace
-- mode. Icons are plain Unicode (or none), rather than its default Nerd Font ones, which
-- show as boxes in terminals w/o such a font.
local ok, render_markdown = pcall(require, 'render-markdown')
if not ok then
    return -- not installed yet (`nvim +PlugInstall +qall`)
end

-- Callout labels w/ a plain bullet instead of their icon, e.g. '󰋽 Note' -> '• Note'
-- (not w/o one, as labels narrower than the '[!NOTE]' they cover leave some of it showing).
local callouts = {}
for name, callout in pairs(render_markdown.default.callout) do
    callouts[name] = { rendered = (callout.rendered:gsub('^[\128-\255]+ ', '• ')) }
end

render_markdown.setup({
    -- Every mode except insert / replace (incl. their Ctrl-O commands, mode 'niI' etc),
    -- so it doesn't flash raw text e.g. while waiting for an operator's motion.
    render_modes = {
        'n', 'no', 'nov', 'noV', 'no\22', 'v', 'V', '\22', 's', 'S', '\19',
        'c', 'r', 'rm', 'r?', 't',
    },
    -- Don't show the cursor line raw.
    anti_conceal = { enabled = false },
    win_options = { concealcursor = { rendered = 'nvc' } },
    -- Keeps the sign column (heading / code block icons there) as Vim has it.
    sign = { enabled = false },
    -- These need a converter (utftex / latex2text), or tree-sitter parsers Neovim doesn't
    -- come with.
    latex = { enabled = false },
    html = { enabled = false },
    yaml = { enabled = false },
    heading = { icons = { '# ', '## ', '### ', '#### ', '##### ', '###### ' } },
    checkbox = {
        unchecked = { icon = '☐' },
        checked = { icon = '☑' },
        custom = { todo = { rendered = '◐' } },
    },
    -- Icons before links (the URL of [text](url) is still hidden).
    link = { enabled = false },
    callout = callouts,
})
