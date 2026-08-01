<?php

namespace App\Service;

use League\CommonMark\Environment\Environment;
use League\CommonMark\Extension\CommonMark\CommonMarkCoreExtension;
use League\CommonMark\Extension\Table\TableExtension;
use League\CommonMark\MarkdownConverter;

/**
 * Renders the project's own markdown docs (`docs/*.md`) for display in the app.
 *
 * The point is a single source of truth: the roadmap and the handover notes are
 * files in the repository — versioned, diffable, and the same text a developer
 * or an AI agent reads — and the web page is a *view* of them rather than a
 * hand-maintained copy that drifts out of date. The previous roadmap page was
 * exactly that copy, and it was stale within a session.
 *
 * HTML in the source is **not** allowed through. These files are trusted (they
 * are in the repo, not user input), but the renderer has no reason to need it,
 * and leaving raw HTML enabled would turn any future "render a doc" reuse into
 * an injection point.
 */
final class MarkdownDocService
{
    /** Documents this service will serve, slug => filename under docs/. */
    private const DOCS = [
        'roadmap' => 'ROADMAP.md',
        'state' => 'PROJECT_STATE.md',
        // The shipped-session log, split out of ROADMAP.md 2026-08-01. Served on
        // its own route so /roadmap stays a 113-line page instead of a 2 000-line one.
        'history' => 'HISTORY.md',
    ];

    public function __construct(private readonly string $projectDir)
    {
    }

    /** @return string|null rendered HTML, or null when the document is missing */
    public function render(string $slug): ?string
    {
        $markdown = $this->read($slug);
        if ($markdown === null) {
            return null;
        }

        $environment = new Environment([
            'html_input' => 'escape',
            'allow_unsafe_links' => false,
        ]);
        $environment->addExtension(new CommonMarkCoreExtension());
        // The docs lean on tables for the tier and level matrices.
        $environment->addExtension(new TableExtension());

        try {
            return (new MarkdownConverter($environment))->convert($markdown)->getContent();
        } catch (\Throwable) {
            return null;
        }
    }

    /** Last-modified time, so the page can say how fresh it is. */
    public function updatedAt(string $slug): ?\DateTimeImmutable
    {
        $path = $this->path($slug);
        if ($path === null || !is_file($path)) {
            return null;
        }

        $stamp = filemtime($path);

        return $stamp === false ? null : (new \DateTimeImmutable())->setTimestamp($stamp);
    }

    private function read(string $slug): ?string
    {
        $path = $this->path($slug);
        if ($path === null || !is_file($path) || !is_readable($path)) {
            return null;
        }

        $contents = file_get_contents($path);

        return $contents === false ? null : $contents;
    }

    /**
     * Resolves a slug to a path. Only the allow-list above can be reached — the
     * slug never becomes part of a filename, so no traversal is possible.
     */
    private function path(string $slug): ?string
    {
        $file = self::DOCS[$slug] ?? null;

        return $file === null ? null : $this->projectDir . '/docs/' . $file;
    }
}
