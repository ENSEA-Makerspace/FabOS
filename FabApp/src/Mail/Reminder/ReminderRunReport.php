<?php

namespace App\Mail\Reminder;

/**
 * What one reminder run did, in enough detail for an admin staring at an empty
 * inbox to tell *why* it was empty — nothing was due, the kind is switched off,
 * mail isn't configured, or everyone had already been told.
 */
final class ReminderRunReport
{
    /** @var array<string, array{sent: int, skipped: int, suppressed: int, would_send: int}> */
    private array $kinds = [];

    /** @var string[] */
    private array $disabled = [];

    /** @var array<string, string> kind => error */
    private array $failures = [];

    /** @var array<int, array{kind: string, recipient: string, template: string, key: string}> */
    private array $preview = [];

    private bool $operational = true;

    public function __construct(public readonly bool $dryRun = false)
    {
    }

    public function markNotOperational(): void
    {
        $this->operational = false;
    }

    public function isOperational(): bool
    {
        return $this->operational;
    }

    public function markDisabled(string $kind): void
    {
        $this->disabled[] = $kind;
    }

    public function markFailed(string $kind, string $error): void
    {
        $this->failures[$kind] = $error;
    }

    public function recordSent(string $kind, ReminderCandidate $candidate): void
    {
        $this->bump($kind, 'sent');
        $this->remember($kind, $candidate);
    }

    public function recordWouldSend(string $kind, ReminderCandidate $candidate): void
    {
        $this->bump($kind, 'would_send');
        $this->remember($kind, $candidate);
    }

    /** Already claimed by an earlier run — the normal case on a busy timer. */
    public function recordSkipped(string $kind): void
    {
        $this->bump($kind, 'skipped');
    }

    /** Claimed, then refused by the Mailer: opted out, or an unusable address. */
    public function recordSuppressed(string $kind): void
    {
        $this->bump($kind, 'suppressed');
    }

    /** @return array<string, array{sent: int, skipped: int, suppressed: int, would_send: int}> */
    public function kinds(): array
    {
        return $this->kinds;
    }

    /** @return string[] */
    public function disabled(): array
    {
        return $this->disabled;
    }

    /** @return array<string, string> */
    public function failures(): array
    {
        return $this->failures;
    }

    /** @return array<int, array{kind: string, recipient: string, template: string, key: string}> */
    public function preview(): array
    {
        return $this->preview;
    }

    public function totalSent(): int
    {
        return array_sum(array_column($this->kinds, 'sent'));
    }

    public function totalWouldSend(): int
    {
        return array_sum(array_column($this->kinds, 'would_send'));
    }

    public function totalSkipped(): int
    {
        return array_sum(array_column($this->kinds, 'skipped'));
    }

    public function totalSuppressed(): int
    {
        return array_sum(array_column($this->kinds, 'suppressed'));
    }

    private function bump(string $kind, string $bucket): void
    {
        $this->kinds[$kind] ??= ['sent' => 0, 'skipped' => 0, 'suppressed' => 0, 'would_send' => 0];
        ++$this->kinds[$kind][$bucket];
    }

    private function remember(string $kind, ReminderCandidate $candidate): void
    {
        // Bounded: a first-run sweep of a large lab could otherwise build a list
        // nobody is going to read anyway.
        if (count($this->preview) >= 50) {
            return;
        }

        $this->preview[] = [
            'kind' => $kind,
            'recipient' => $candidate->recipient(),
            'template' => $candidate->template,
            'key' => $candidate->key,
        ];
    }
}
