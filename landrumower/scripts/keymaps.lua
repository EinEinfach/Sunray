vim.keymap.set("n", "<leader>d", function()
    local cwd = vim.fn.getcwd()  -- aktuelles Arbeitsverzeichnis
    local current_file = vim.api.nvim_buf_get_name(0)  -- kompletter Pfad zur aktuellen Datei
  
    -- Check: ist main.py geöffnet?
    if not current_file:match("main%.py$") then
      vim.notify("❌ current file is not main.py", vim.log.levels.ERROR)
      return
    end
  
    -- relativer Pfad zu main.py
    local main_py = vim.fn.fnamemodify(current_file, ":.")
  
    -- deploy.sh erwartet `main.py` als Argument
    local deploy_script = cwd .. "/../scripts/deploy.sh"
  
    if vim.fn.filereadable(deploy_script) == 0 then
      vim.notify("❌ deploy.sh nicht gefunden!", vim.log.levels.ERROR)
      return
    end
  
    -- Info-Startmeldung
    vim.notify("🚀 Starte Deployment von " .. main_py, vim.log.levels.INFO)
  
    -- ausführen
    vim.fn.jobstart({ "bash", deploy_script, main_py }, {
      stdout_buffered = true,
      stderr_buffered = true,
      on_stdout = function(_, data)
        if data then
          for _, line in ipairs(data) do
            if line ~= "" then
              vim.notify("📤 " .. line, vim.log.levels.INFO)
            end
          end
        end
      end,
      on_stderr = function(_, data)
        if data then
          for _, line in ipairs(data) do
            if line ~= "" then
              vim.notify("⚠️ " .. line, vim.log.levels.WARN)
            end
          end
        end
      end,
      on_exit = function(_, code)
        if code == 0 then
          vim.notify("✅ Deploy erfolgreich!", vim.log.levels.INFO)
        else
          vim.notify("❌ Deploy fehlgeschlagen (Exit " .. code .. ")", vim.log.levels.ERROR)
        end
      end,
    })
  end, { desc = "Deploy main.py to Pico" })