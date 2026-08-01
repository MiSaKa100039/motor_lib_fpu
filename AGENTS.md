# Project Codex Instructions

## Language and Encoding

- Treat source files in this project as UTF-8 by default and keep them as UTF-8 without BOM unless the user explicitly requests a different encoding.
- New comments in `.c`, `.h`, `.cpp`, `.hpp`, `.s`, `.S`, `.ld`, `.cmake`, and `CMakeLists.txt` should default to Chinese.
- Chinese comments are allowed directly in source files and are not restricted to Markdown documents.
- Do not create Chinese file names, directory names, macro names, variable names, function names, or type names.

## Comment Preservation

- Do not delete, overwrite, or rewrite existing Chinese comments unless the user explicitly asks for comment cleanup, terminology unification, or garbled-text repair.
- When a file already contains Chinese comments, preserve the original meaning and prefer incremental additions instead of broad rewrites.
- If garbled Chinese text is found, keep it in place and report it first unless the user explicitly requests repair.
- Avoid formatting or bulk-rewrite operations that could indirectly remove existing comments.

## Comment-Only Editing Rules

- For comment-annotation tasks, modify comments only and do not change macro values, function signatures, struct fields, control parameters, register settings, or runtime logic.
- Prefer local insertions and small patches instead of whole-file rewrites to reduce the risk of losing nearby comments.
- In ISR, HAL, control-loop, manager, and other timing-sensitive code, keep comments concise and focus on safety constraints, timing assumptions, hardware limits, and non-obvious control logic.
- Do not add obvious narration comments that only restate the code.

## Embedded C/C++ Boundaries

- Do not perform large unrelated refactors.
- Do not change public APIs without explaining compatibility impact.
- Keep ISR, HAL, control-loop, manager, FSM, and user API layers clearly separated in motor-control code.
- Avoid adding dynamic allocation unless explicitly requested.
- Avoid blocking operations inside ISR or fast control-loop code.
- For STM32 or LC32 generated code regions, do not modify tool-generated sections unless necessary for the requested task.
