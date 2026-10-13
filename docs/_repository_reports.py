"""Expose repository reports as generated pages without copying historical data."""
from __future__ import annotations

import posixpath
import re
from pathlib import Path

from mkdocs.structure.files import File

ROOT = Path(__file__).resolve().parents[1]


def on_files(files, config):
    sources = {p: "reports/"+p.name for p in (ROOT/"results").glob("*.md")}
    evidence = ROOT / "results/petal_angle1_blend030_20261003"
    for source in evidence.iterdir():
        if source.is_file():
            sources[source] = "reports/petal_angle1_blend030_20261003/" + source.name
    sources[ROOT/"assets/interfaces/petal_guidance/README.md"] = "reports/petal_guidance.md"
    for source, uri in sources.items():
        if source.suffix != ".md":
            files.append(File.generated(config, uri, abs_src_path=str(source)))
            continue
        def rewrite(match, source=source, uri=uri):
            image, label, href = match.groups()
            if "://" in href or href.startswith("#"):
                return match[0]
            target = (source.parent/href.split("#")[0]).resolve()
            if target in sources:
                relative = posixpath.relpath(sources[target], posixpath.dirname(uri))
                return f"{image}[{label}]({relative})"
            if target.is_relative_to(ROOT/"docs"):
                relative = posixpath.relpath(str(target.relative_to(ROOT/"docs")), posixpath.dirname(uri))
                return f"{image}[{label}]({relative})"
            # runs/ is intentionally excluded from Git/CI. Publish report prose
            # and preserve figure/log locations without creating broken links.
            location = str(target.relative_to(ROOT)) if target.is_relative_to(ROOT) else href
            return f"{'图件：' if image else ''}{label}（仓库文件 `{location}`）"
        content = re.sub(r"(!?)\[([^\]]+)\]\(([^)]+)\)", rewrite, source.read_text())
        files.append(File.generated(config, uri, content=content))
    return files


def on_page_markdown(markdown, page, config, files):
    # 源文档使用可在仓库中打开的相对链接，构建站点时转向生成的历史证据页。
    if page.file.src_uri in {"control_research.md", "models_interfaces.md",
                             "historical_evidence.md", "development_plan.md",
                             "control_main_results.md"}:
        return markdown.replace("(../results/", "(reports/")
    return markdown
