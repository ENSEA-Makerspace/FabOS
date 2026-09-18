<?php

namespace App\Command;

use App\Service\SiteSettingService;
use App\Service\ThemeManager;
use App\Theme\ThemePreview;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\RequestStack;

/**
 * S167 — la sonde de la publication atomique et de la garde d'aperçu.
 *
 * 🔴 **Ce qu'elle mesure vraiment, et ce qu'elle ne peut pas mesurer.** La garde
 * de l'aperçu a deux moitiés : « un administrateur voit le brouillon » et
 * « personne d'autre ne le voit ». La console n'a pas de session, donc elle ne
 * peut prouver que la SECONDE — qui est celle qui compte, parce que c'est celle
 * dont l'échec est une fuite. La première est consignée comme ligne à regarder à
 * l'écran, pas déclarée verte ici.
 *
 * ✅ **La publication, elle, se mesure entièrement** : le refus sur un logo
 * disparu, et surtout le fait que RIEN n'est écrit quand il refuse.
 *
 * ⚠️ **Le brouillon et les réglages publiés sont capturés AVANT et remis
 * APRÈS**, quoi qu'il arrive. Une sonde qui manipule le thème d'une installation
 * réelle ne doit pas dépendre du comportement qu'elle mesure pour être
 * inoffensive.
 */
#[AsCommand(name: 'app:s167:theme-probe', description: 'S167 : prouve qu\'un aperçu de brouillon est refusé sans droits d\'admin, et qu\'une publication qui échoue n\'écrit RIEN. Remet tout en place.')]
final class S167ThemePublishProbeCommand extends Command
{
    private const KEYS = ['org_name', 'venue_label', 'site_primary_color', 'site_logo_path'];

    public function __construct(
        private readonly ThemeManager $themes,
        private readonly SiteSettingService $settings,
        private readonly ThemePreview $preview,
        private readonly RequestStack $requests,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];

        $draftBefore = $this->themes->draft();
        $publishedBefore = [];
        foreach (self::KEYS as $key) {
            $publishedBefore[$key] = (string) $this->settings->get($key);
        }

        try {
            $io->section('1. 🔴 Sans droits d\'administrateur, le brouillon NE FUIT PAS');
            // ⚠️ La console n'a pas de jeton de sécurité : c'est exactement la
            // situation d'un visiteur, et c'est la moitié de la garde dont
            // l'échec serait une fuite.
            $this->requests->push(Request::create('/?theme=draft&theme_mode=dark'));
            $this->check($io, $failures, '🔴 `?theme=draft` seul ne suffit PAS', !$this->preview->isPreviewing());
            $this->check($io, $failures, 'et le mode forcé est donc nul', $this->preview->forcedMode() === null);
            $this->requests->pop();

            $this->requests->push(Request::create('/'));
            $this->check($io, $failures, 'sans le paramètre non plus', !$this->preview->isPreviewing());
            $this->requests->pop();

            $io->section('2. 🔴 Un logo disparu REFUSE la publication');
            // ⚠️ La médiathèque interdit déjà de supprimer une image référencée
            // par le brouillon. Cette garde couvre ce que l'autre ne voit pas :
            // une suppression en base à la main, une restauration, un fichier
            // parti du disque.
            $this->themes->saveDraft([
                'orgName' => $draftBefore['orgName'] ?: 'FabOS',
                'venueLabel' => $draftBefore['venueLabel'] ?: 'FabLab',
                'primaryColor' => '',
                'logoPath' => str_repeat('a', 32),
            ]);

            $refused = null;
            try {
                $this->themes->publish();
            } catch (\InvalidArgumentException $e) {
                $refused = $e->getMessage();
            }

            $this->check($io, $failures, '🔴 la publication REFUSE', $refused !== null);
            if ($refused !== null) {
                $io->writeln('   « ' . $refused . ' »');
                // ⚠️ « Rien n'a été publié » doit être DIT : sinon on republie,
                // ou pire, on croit que c'est passé.
                $this->check($io, $failures, 'et le message dit que rien n\'a été publié', str_contains($refused, 'Rien'));
            }

            $io->section('3. 🔴 Et RIEN n\'a été écrit — c\'est tout l\'intérêt');
            $drift = [];
            foreach (self::KEYS as $key) {
                if ((string) $this->settings->get($key) !== $publishedBefore[$key]) {
                    $drift[] = $key;
                }
            }
            $io->writeln('   réglages publiés modifiés : ' . ($drift === [] ? '(aucun)' : implode(', ', $drift)));
            $this->check($io, $failures, '🔴 les quatre réglages publiés sont INTACTS', $drift === []);

            $io->section('4. Une publication valide passe, et les quatre bougent ENSEMBLE');
            $this->themes->saveDraft([
                'orgName' => $draftBefore['orgName'] ?: 'FabOS',
                'venueLabel' => $draftBefore['venueLabel'] ?: 'FabLab',
                'primaryColor' => '',
                'logoPath' => '',
            ]);
            $this->themes->publish();
            $this->check($io, $failures, 'le nom publié suit le brouillon', (string) $this->settings->get('org_name') === ($draftBefore['orgName'] ?: 'FabOS'));
            $this->check($io, $failures, 'et le logo aussi', (string) $this->settings->get('site_logo_path') === '');
        } finally {
            // 🔴 Remise en état inconditionnelle, publiés ET brouillon.
            $this->settings->transactional(function () use ($publishedBefore): void {
                foreach ($publishedBefore as $key => $value) {
                    $this->settings->set($key, $value);
                }
            });
            $this->themes->saveDraft($draftBefore);
        }

        $io->section('5. Tout est remis comme avant');
        $restored = true;
        foreach (self::KEYS as $key) {
            $restored = $restored && (string) $this->settings->get($key) === $publishedBefore[$key];
        }
        $this->check($io, $failures, 'les réglages publiés sont revenus', $restored);
        $this->check($io, $failures, 'le brouillon aussi', $this->themes->draft() == $draftBefore);

        $io->section('La moitié POSITIVE se mesure AILLEURS, et elle l\'est');
        // ⚠️ Elle demande une session d'administrateur, que la console n'a pas —
        // mais `app:render` en a une. Mesuré le 2026-09-18 :
        //   app:render "/?theme=draft&theme_mode=dark"
        //     → <html lang="en" data-theme="dark" data-theme-locked="1">
        //   app:render "/"
        //     → <html lang="en">                       (aucune fuite)
        //   app:render "/admin/themes?preview=1"
        //     → quatre cadres, vers /?theme=draft et /machines?theme=draft
        $io->writeln('   `app:render` a la session que la console n\'a pas. Commandes dans');
        $io->writeln('   l\'en-tête de cette section, résultats consignés dans ROADMAP.md.');

        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }

        $io->success('Sonde S167 verte. Thème remis à son état de départ.');

        return Command::SUCCESS;
    }

    /** @param list<string> $failures */
    private function check(SymfonyStyle $io, array &$failures, string $what, bool $ok): void
    {
        $io->writeln(($ok ? '   <info>✓</info> ' : '   <error>✗</error> ') . $what);
        if (!$ok) {
            $failures[] = $what;
        }
    }
}
