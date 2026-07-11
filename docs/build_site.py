"""
Static site generator for the bmech docs site.

Converts docs/bmech/<family>/*.md into styled HTML pages matching the
site's existing dark theme (see docs/bmech/index.html), plus a category
landing page per family. Run this after any edit to a .md file under
docs/bmech/ and commit the regenerated HTML alongside it — this is a
convention, not a CI check; see docs/bmech/CONVENTIONS.md's "the live
site is generated from these docs" section.

Usage: python docs/build_site.py  (or any other Python 3 interpreter —
this is pure standard library, no pip installs required, since it needs
to run with whatever plain Python is on hand.)

The markdown converter below is NOT a general CommonMark implementation
— it only handles the subset actually used across docs/bmech/**/*.md
(checked empirically before writing this): headers, bold, inline code,
fenced code blocks, links (rewritten from .md to .html — the source docs
link to each other as .md since that's what renders on GitHub directly;
the generated site needs those same links pointed at the generated .html
instead), tables, unordered/ordered lists, blockquotes, paragraphs,
horizontal rules. No images, no italics, no nested lists — none of the
existing docs use them, so there's nothing to test that behavior against.
"""

import re
import os
import shutil

DOCS_ROOT = os.path.dirname(os.path.abspath(__file__))
BMECH_ROOT = os.path.join(DOCS_ROOT, "bmech")
IMG_DIR = os.path.join(BMECH_ROOT, "assets", "img")

FAMILY_TITLES = {
    "gears": "Gears",
    "fasteners": "Fasteners",
    "bearings": "Bearings",
    "springs": "Springs",
    "ratchets": "Ratchets",
}

# Short, end-user-facing explanations of each family's shared settings —
# hand-written, not extracted from the .md docs, since those are written
# for contributors (implementation detail, boolean-solver notes, etc.)
# and this is deliberately the opposite: what a normal user needs to know
# to set the sliders sensibly, nothing about how the generator is built.
UNITS_BLURB = {
    "gears": (
        "Every gear shares two settings that have to match for two gears "
        "to mesh: <strong>module</strong> (tooth size — pick one module "
        "and use it for every gear in a train) and "
        "<strong>pressure angle</strong> (tooth shape, usually 20°). "
        "Everything else — tooth count, width, bore — is specific to "
        "that one gear."
    ),
    "fasteners": (
        "Threads are described by four numbers that all have to match "
        "for a bolt and nut to fit together: <strong>thread diameter</strong> "
        "(the nominal size, e.g. M8), <strong>pitch</strong> (distance "
        "between threads), <strong>flank angle</strong> (thread shape, "
        "usually 60°), and <strong>truncation</strong> (how flat the "
        "crest and root are)."
    ),
    "bearings": (
        "Set the <strong>bore diameter</strong> (for the axle) and "
        "<strong>outer diameter</strong> (for the housing), then choose "
        "how many balls to use — more balls means a larger bearing for "
        "the same ball size. The overhang angle controls how much "
        "support-free clearance the ball pockets get when printing."
    ),
    "springs": (
        "Both spring generators build a flat coiled ribbon. "
        "<strong>Strip width</strong> and <strong>strip thickness</strong> "
        "control the ribbon's own cross-section; inner/outer radius and "
        "turn count control the size and tightness of the coil."
    ),
    "ratchets": (
        "Ratchets use the same sizing convention as gears: set "
        "<strong>module</strong> (tooth size) and <strong>tooth count</strong>, "
        "or switch to outer-diameter mode and let module be worked out "
        "automatically. Tooth depth can be automatic or set by hand."
    ),
}

# Whether a family supports Match Target at all, and if so, a short
# description of what it does — no implementation detail (no mention of
# WindowManager pointers, poll callbacks, or freezing behavior; that's
# contributor-facing content, not something an end user needs).
MATCH_TARGET_BLURB = {
    "gears": (
        "Pick another gear as a <strong>Match Target</strong> and its "
        "module and pressure angle are copied over automatically, so "
        "the new gear is guaranteed to mesh with it."
    ),
    "fasteners": (
        "Pick a bolt or nut as a <strong>Match Target</strong> and its "
        "thread dimensions are copied over automatically, so the two "
        "are guaranteed to fit together."
    ),
    "bearings": None,
    "springs": None,
    "ratchets": None,
}

# Primitive doc filename (without .md) -> showcase render filename (with
# .png). Doc filenames match the Python module name; render filenames
# match the operator's own bl_idname — these don't always agree (e.g. a
# module named straight_rack.py registers as object.add_rack), so this
# has to be an explicit table, not derived automatically.
PRIMITIVE_IMAGES = {
    "annulus_gear": "annulus_gear.png",
    "bevel_gear": "bevel_gear.png",
    "cluster_gear": "add_cluster_gear.png",
    "compound_gear": "add_compound_gear.png",
    "helical_annulus_gear": "helical_annulus_gear.png",
    "helical_gear": "helical_gear.png",
    "helical_planetary_gear_set": "helical_planetary_gear_set.png",
    "helical_rack": "helical_rack.png",
    "herringbone_annulus_gear": "herringbone_annulus_gear.png",
    "herringbone_gear": "herringbone_gear.png",
    "herringbone_planetary_gear_set": "herringbone_planetary_gear_set.png",
    "herringbone_rack": "herringbone_rack.png",
    "planetary_gear_set": "planetary_gear_set.png",
    "spur_gear": "add_spur_gear.png",
    "straight_rack": "add_rack.png",
    "hex_bolt": "hex_bolt.png",
    "hex_nut": "hex_nut.png",
    "threaded_container": "threaded_container.png",
    "threaded_fastener": "add_threaded_fastener.png",
    "threaded_lid": "threaded_lid.png",
    "ball_bearing": "add_ball_bearing.png",
    "hairspring": "add_hairspring.png",
    "serpentine_spring": "add_serpentine_spring.png",
    "internal_ratchet": "add_internal_ratchet.png",
    "ratchet_pawl": "add_ratchet_mechanism.png",
}


# ── Minimal markdown -> HTML converter ─────────────────────────────────────

def _inline(text):
    # Escape HTML-unsafe characters FIRST, before any markdown substitution
    # inserts real tags — otherwise literal `<`/`>`/`&` from the source
    # markdown (e.g. `dedendum_radius <= 0` in an inline code span) leak
    # into the output unescaped, and escaping afterward would instead
    # mangle the tags this function just inserted.
    text = text.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')

    # Pull code spans out into placeholders BEFORE bold/italic/link
    # processing, and substitute them back in as the very last step. Code
    # span content is real code — asterisks in there (e.g. `hand_sign * z
    # * tan(helix_angle)`, a literal multiplication) are not markdown
    # emphasis markers, but a plain substitution pass can't tell the
    # difference once the backticks are gone, since it just re-scans
    # whatever text is sitting there. Protecting the content behind a
    # placeholder until after those passes run is the standard fix.
    code_spans = []

    def _stash_code(m):
        code_spans.append(m.group(1))
        return '\x00CODE%d\x00' % (len(code_spans) - 1)

    text = re.sub(r'`([^`]+)`', _stash_code, text)

    text = re.sub(r'\*\*([^*]+)\*\*', r'<strong>\1</strong>', text)
    # Bold runs first and consumes every ** pair, so any single * left by
    # this point is genuine italic emphasis, not a stray half of a bold marker.
    text = re.sub(r'\*([^*]+)\*', r'<em>\1</em>', text)

    def _link(m):
        label, href = m.group(1), m.group(2)
        if href.startswith(('http://', 'https://', 'mailto:')):
            return '<a href="%s">%s</a>' % (href, label)
        # README.md and CONVENTIONS.md don't get their own public page —
        # they're internal/contributor-facing docs (family-wide shared
        # conventions, cross-family rules), not something an end user
        # downloading the extension needs. Rather than link to a page that
        # doesn't exist, demote these to plain text: keep the label,
        # drop the link.
        basename = href.split('/')[-1].split('#')[0]
        if basename in ('README.md', 'CONVENTIONS.md'):
            return label
        href = re.sub(r'\.md(#|$)', r'.html\1', href)
        return '<a href="%s">%s</a>' % (href, label)

    text = re.sub(r'\[([^\]]+)\]\(([^)]+)\)', _link, text)

    for idx, code in enumerate(code_spans):
        text = text.replace('\x00CODE%d\x00' % idx, '<code>%s</code>' % code)
    return text


def _render_table(table_lines):
    rows = [l.strip().strip('|').split('|') for l in table_lines if l.strip()]
    rows = [[c.strip() for c in r] for r in rows]
    header, _, *body = rows
    out = ['<div class="table-wrap"><table>', '<thead><tr>']
    for c in header:
        out.append('<th>%s</th>' % _inline(c))
    out.append('</tr></thead><tbody>')
    for r in body:
        out.append('<tr>' + ''.join('<td>%s</td>' % _inline(c) for c in r) + '</tr>')
    out.append('</tbody></table></div>')
    return '\n'.join(out)


def markdown_to_html(md_text):
    lines = md_text.split('\n')
    out = []
    i, n = 0, len(lines)
    para_buf = []

    def flush_para():
        if para_buf:
            out.append('<p>%s</p>' % _inline(' '.join(para_buf)))
            para_buf.clear()

    while i < n:
        line = lines[i]

        if line.strip().startswith('```'):
            flush_para()
            i += 1
            code_lines = []
            while i < n and not lines[i].strip().startswith('```'):
                code_lines.append(lines[i])
                i += 1
            i += 1
            escaped = ('\n'.join(code_lines)
                       .replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;'))
            out.append('<pre><code>%s</code></pre>' % escaped)
            continue

        m = re.match(r'^(#{1,4})\s+(.*)', line)
        if m:
            flush_para()
            level = len(m.group(1))
            out.append('<h%d>%s</h%d>' % (level, _inline(m.group(2)), level))
            i += 1
            continue

        if re.match(r'^-{3,}\s*$', line):
            flush_para()
            out.append('<hr>')
            i += 1
            continue

        if ('|' in line and i + 1 < n
                and re.match(r'^\s*\|?[\s:|-]+\|?\s*$', lines[i + 1]) and '-' in lines[i + 1]):
            flush_para()
            table_lines = [line, lines[i + 1]]
            i += 2
            while i < n and '|' in lines[i] and lines[i].strip():
                table_lines.append(lines[i])
                i += 1
            out.append(_render_table(table_lines))
            continue

        if line.strip().startswith('>'):
            flush_para()
            quote_lines = []
            while i < n and lines[i].strip().startswith('>'):
                quote_lines.append(re.sub(r'^\s*>\s?', '', lines[i]))
                i += 1
            out.append('<blockquote><p>%s</p></blockquote>' % _inline(' '.join(quote_lines)))
            continue

        if re.match(r'^\s*-\s+', line):
            flush_para()
            items = []
            while i < n and re.match(r'^\s*-\s+', lines[i]):
                item_text = re.sub(r'^\s*-\s+', '', lines[i])
                i += 1
                while (i < n and lines[i].strip()
                       and not re.match(r'^\s*-\s+', lines[i])
                       and not re.match(r'^#{1,4}\s', lines[i])):
                    item_text += ' ' + lines[i].strip()
                    i += 1
                items.append(item_text)
            out.append('<ul>' + ''.join('<li>%s</li>' % _inline(it) for it in items) + '</ul>')
            continue

        if re.match(r'^\s*\d+\.\s+', line):
            flush_para()
            items = []
            while i < n and re.match(r'^\s*\d+\.\s+', lines[i]):
                item_text = re.sub(r'^\s*\d+\.\s+', '', lines[i])
                i += 1
                while (i < n and lines[i].strip()
                       and not re.match(r'^\s*\d+\.\s+', lines[i])
                       and not re.match(r'^#{1,4}\s', lines[i])):
                    item_text += ' ' + lines[i].strip()
                    i += 1
                items.append(item_text)
            out.append('<ol>' + ''.join('<li>%s</li>' % _inline(it) for it in items) + '</ol>')
            continue

        if not line.strip():
            flush_para()
            i += 1
            continue

        para_buf.append(line.strip())
        i += 1

    flush_para()
    return '\n'.join(out)


# ── Page template (matches docs/bmech/index.html's dark theme) ────────────

PAGE_TEMPLATE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>{title} — bmech</title>
<style>
  :root {{
    --bg: #16181d;
    --panel: #1e2128;
    --text: #e8e8e8;
    --muted: #9aa0aa;
    --accent: #6fb1ff;
    --border: #2c3038;
  }}
  * {{ box-sizing: border-box; }}
  body {{
    margin: 0;
    background: var(--bg);
    color: var(--text);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    line-height: 1.6;
  }}
  .breadcrumb {{
    max-width: 860px;
    margin: 0 auto;
    padding: 1.5rem 1.5rem 0;
    color: var(--muted);
    font-size: 0.9rem;
  }}
  .breadcrumb a {{ color: var(--muted); }}
  .breadcrumb a:hover {{ color: var(--accent); }}
  header {{
    padding: 2rem 1.5rem 3rem;
    text-align: center;
  }}
  header h1 {{ font-size: 2.2rem; margin: 0 0 0.5rem; }}
  .hero-img {{
    display: block;
    max-width: 420px;
    width: 100%;
    height: auto;
    margin: 1.5rem auto 0;
    border-radius: 10px;
    border: 1px solid var(--border);
    background: var(--panel);
  }}
  main {{
    max-width: 860px;
    margin: 0 auto;
    padding: 0 1.5rem 3rem;
  }}
  h2 {{
    font-size: 1.3rem;
    border-left: 3px solid var(--accent);
    padding-left: 0.75rem;
    margin-top: 2.5rem;
  }}
  h3 {{ color: var(--accent); font-size: 1.05rem; }}
  p, li {{ color: var(--text); }}
  code {{
    background: var(--panel);
    padding: 0.15rem 0.4rem;
    border-radius: 4px;
    font-size: 0.9em;
  }}
  pre {{
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 1rem;
    overflow-x: auto;
  }}
  pre code {{ background: none; padding: 0; }}
  blockquote {{
    border-left: 3px solid var(--border);
    margin: 0;
    padding-left: 1rem;
    color: var(--muted);
    font-style: italic;
  }}
  a {{ color: var(--accent); }}
  .table-wrap {{ overflow-x: auto; }}
  table {{ border-collapse: collapse; width: 100%; margin: 1rem 0; }}
  th, td {{
    border: 1px solid var(--border);
    padding: 0.5rem 0.7rem;
    text-align: left;
    font-size: 0.92rem;
  }}
  th {{ background: var(--panel); color: var(--accent); }}
  .grid {{
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
    gap: 1rem;
    margin-top: 1.5rem;
  }}
  .card {{
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: 1.2rem;
    text-decoration: none;
    display: block;
    transition: border-color 0.15s ease;
  }}
  .card:hover {{ border-color: var(--accent); }}
  .card img {{
    width: 100%;
    height: 140px;
    object-fit: cover;
    border-radius: 6px;
    margin-bottom: 0.8rem;
    background: var(--bg);
  }}
  .card h3 {{ margin: 0 0 0.4rem; }}
  .card p {{ margin: 0; color: var(--muted); font-size: 0.9rem; }}
  .section-index {{
    display: flex;
    flex-wrap: wrap;
    gap: 0.6rem;
    margin: 1.5rem 0 2.5rem;
  }}
  .section-index a {{
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 6px;
    padding: 0.45rem 0.9rem;
    font-size: 0.88rem;
    text-decoration: none;
  }}
  .section-index a:hover {{ border-color: var(--accent); }}
  details {{
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 10px;
    margin: 1rem 0;
    scroll-margin-top: 1.5rem;
  }}
  summary {{
    cursor: pointer;
    padding: 1rem 1.2rem;
    font-size: 1.15rem;
    font-weight: 600;
    color: var(--accent);
    list-style: none;
  }}
  summary::-webkit-details-marker {{ display: none; }}
  summary::before {{
    content: '▸';
    display: inline-block;
    width: 1em;
    transition: transform 0.15s ease;
  }}
  details[open] summary::before {{ transform: rotate(90deg); }}
  details > *:not(summary) {{
    padding: 0 1.2rem 1.2rem;
  }}
  details .grid {{ margin-top: 0; }}
  footer {{
    border-top: 1px solid var(--border);
    text-align: center;
    padding: 2rem 1.5rem;
    color: var(--muted);
    font-size: 0.9rem;
  }}
  footer a {{ color: var(--muted); }}
</style>
</head>
<body>
<div class="breadcrumb">{breadcrumb}</div>
<header>
  <h1>{heading}</h1>
  {hero_img}
</header>
<main>
{body}
</main>
<footer>
  <p><a href="https://github.com/lettvce/bmech">GitHub</a> ·
     <a href="mailto:blakewysocki421@gmail.com">Contact</a></p>
</footer>
<script>
// Opens the <details> section matching the URL's #hash, so clicking an
// index link actually shows that section instead of just scrolling next
// to a still-collapsed one. Harmless no-op on pages with no <details> or
// no hash. No framework, no dependency — a handful of lines is all this
// needs.
(function () {{
  function openFromHash() {{
    var id = window.location.hash.slice(1);
    if (!id) return;
    var el = document.getElementById(id);
    if (el && el.tagName === 'DETAILS') el.open = true;
  }}
  window.addEventListener('hashchange', openFromHash);
  openFromHash();
}})();
</script>
</body>
</html>
"""


def render_page(title, heading, breadcrumb, body_html, hero_img_rel=None):
    hero_html = ('<img class="hero-img" src="%s" alt="%s render">' % (hero_img_rel, heading)
                 if hero_img_rel else '')
    return PAGE_TEMPLATE.format(
        title=title, heading=heading, breadcrumb=breadcrumb,
        body=body_html, hero_img=hero_html,
    )


def first_paragraph(md_text):
    """
    First non-heading paragraph of a doc, as PLAIN TEXT (markdown syntax
    stripped, not converted to HTML), for card-grid summaries. Plain text
    because it gets truncated to a short teaser length — running that
    truncated fragment through _inline() afterward is what produced
    literal "[annulus_gear.md](a" garbage in the card summaries: a link
    or code span cut off mid-syntax just doesn't match _inline()'s
    regexes, so the raw markdown leaks through untouched. Stripping the
    syntax to plain words FIRST sidesteps that entirely — there's no
    markup left to split.
    """
    for block in re.split(r'\n\s*\n', md_text):
        block = block.strip()
        if block and not block.startswith('#') and not block.startswith('`'):
            text = re.sub(r'\s+', ' ', block)
            text = re.sub(r'`([^`]+)`', r'\1', text)
            text = re.sub(r'\*\*([^*]+)\*\*', r'\1', text)
            text = re.sub(r'\*([^*]+)\*', r'\1', text)
            text = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', text)
            text = text.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')
            return text[:160]
    return ''


def strip_leading_h1(md_text):
    """
    Drop the doc's own leading `# Title` line — the page template already
    renders the title prominently in its own <h1> in the header, so
    leaving the markdown's copy in would duplicate it in the body.
    """
    lines = md_text.split('\n', 1)
    if lines and re.match(r'^#\s+', lines[0]):
        return lines[1] if len(lines) > 1 else ''
    return md_text


def remove_stale_conventions_page():
    """
    CONVENTIONS.html used to be generated from CONVENTIONS.md. It isn't
    anymore — README.md/CONVENTIONS.md are internal/contributor-facing
    docs (family-wide shared conventions, cross-family rules), not
    something an end user downloading the extension needs, so neither
    gets a public page now (see _link()'s own handling of links to them).
    This just cleans up a leftover file from a previous run so re-running
    the build after this change doesn't leave a stale, now-unlinked page
    sitting on disk.
    """
    path = os.path.join(BMECH_ROOT, 'CONVENTIONS.html')
    if os.path.exists(path):
        os.remove(path)
        print('removed stale CONVENTIONS.html')


def build():
    remove_stale_conventions_page()

    families = sorted(
        d for d in os.listdir(BMECH_ROOT)
        if os.path.isdir(os.path.join(BMECH_ROOT, d)) and d in FAMILY_TITLES
    )

    for family in families:
        family_dir = os.path.join(BMECH_ROOT, family)
        family_title = FAMILY_TITLES[family]

        primitive_files = sorted(
            f for f in os.listdir(family_dir)
            if f.endswith('.md') and f != 'README.md'
        )

        # ── Per-generator pages ──────────────────────────────────────────
        cards = []
        for fname in primitive_files:
            stem = fname[:-3]
            with open(os.path.join(family_dir, fname), 'r', encoding='utf-8') as f:
                md_text = f.read()

            display_name = stem.replace('_', ' ').title()
            img_name = PRIMITIVE_IMAGES.get(stem)
            img_exists = img_name and os.path.exists(os.path.join(IMG_DIR, img_name))
            img_rel = ('../assets/img/' + img_name) if img_exists else None

            body_html = markdown_to_html(strip_leading_h1(md_text))
            breadcrumb = ('<a href="../index.html">bmech</a> / '
                          '<a href="index.html">%s</a> / %s' % (family_title, display_name))
            html = render_page(display_name, display_name, breadcrumb, body_html, img_rel)
            with open(os.path.join(family_dir, stem + '.html'), 'w', encoding='utf-8') as f:
                f.write(html)

            # first_paragraph() already strips markdown to plain text and
            # HTML-escapes it — don't run it through _inline() too, that
            # would double-escape (&amp; -> &amp;amp;) since there's no
            # markdown syntax left for _inline() to convert.
            summary = first_paragraph(md_text)
            card_img_html = '<img src="../assets/img/%s" alt="">' % img_name if img_exists else ''
            cards.append(
                '<a class="card" href="%s.html">%s<h3>%s</h3><p>%s</p></a>'
                % (stem, card_img_html, display_name, summary)
            )

        # ── Category landing page ────────────────────────────────────────
        # Three collapsible sections (each starts closed — <details> with
        # no `open` attribute — and expands when its <summary> header is
        # clicked, no JS needed for that part), plus a small index at the
        # top linking to each by anchor. No README.md content — that's the
        # family's internal shared-conventions writeup (contributor/AI-
        # agent facing per CLAUDE.md), not something an end user needs;
        # the Units and Target Matching blurbs below are hand-written for
        # end users instead of extracted from it.
        match_target_html = MATCH_TARGET_BLURB.get(family)
        if match_target_html is None:
            match_target_html = "This family doesn't currently support Match Target."

        index_links = [
            '<a href="#primitives">Primitives in this family</a>',
            '<a href="#units-core-parameters">Units and Core Parameters</a>',
            '<a href="#target-matching">Target Matching</a>',
        ]

        body_html = (
            '<nav class="section-index">' + ''.join(index_links) + '</nav>\n'
            '<details id="primitives">\n'
            '<summary>Primitives in this family</summary>\n'
            '<div class="grid">\n' + '\n'.join(cards) + '\n</div>\n'
            '</details>\n'
            '<details id="units-core-parameters">\n'
            '<summary>Units and Core Parameters</summary>\n'
            '<p>' + UNITS_BLURB[family] + '</p>\n'
            '</details>\n'
            '<details id="target-matching">\n'
            '<summary>Target Matching</summary>\n'
            '<p>' + match_target_html + '</p>\n'
            '</details>\n'
        )
        breadcrumb = '<a href="../index.html">bmech</a> / %s' % family_title
        html = render_page(family_title, family_title, breadcrumb, body_html)
        with open(os.path.join(family_dir, 'index.html'), 'w', encoding='utf-8') as f:
            f.write(html)

        print('built %s: %d primitive page(s) + 1 category page' % (family, len(primitive_files)))

    print('done.')


if __name__ == '__main__':
    build()
