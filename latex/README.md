# Multi-file LaTeX Report Template

## Quick start

1. Edit report metadata near the top of `main.tex`.
2. Write each chapter in its own file under `chapters/`.
3. Add or remove `\include{chapters/...}` lines in `main.tex`.
4. Put images in `figures/`.
5. Put BibTeX entries in `references.bib`.

Compile from the template folder with:

```bash
latexmk -xelatex main.tex
```

To remove temporary compilation files:

```bash
latexmk -c
```

On Overleaf, upload the entire folder (or the ZIP file), set `main.tex` as the
main document, and choose **XeLaTeX** as the compiler.

## Navigation behavior

- The table of contents, citations, references, equations, figures, and tables
  are clickable.
- Every page contains **Previous / Contents / Next** navigation and a
  **Back to Contents** link.
- Displayed page numbers use one continuous Arabic sequence so they agree with
  the page-number field in most PDF readers.
- The sidebar outline is generated from chapters and sections.

Some browser PDF viewers do not support PDF `PrevPage` and `NextPage` actions.
The contents links and normal page-number entry still work; Acrobat Reader
supports the full navigation set.

This base version is optimized for English engineering reports and does not
require a Chinese TeX package. If Chinese body text is needed, install a full
TeX distribution and add `\\usepackage[UTF8]{ctex}` to `config.tex`.
