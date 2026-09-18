<?php

namespace App\Command;

use App\Media\SiteMediaLibrary;
use App\Service\ThemeManager;
use App\Theme\ThemePresets;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;
use Symfony\Component\HttpFoundation\File\UploadedFile;

/**
 * S166 — la sonde des préréglages et des variantes de logo.
 *
 * 🔴 **Le piège central, trouvé en LISANT `style.css` et pas en livrant** : sous
 * 576 px, `--spacing-lg`, `--xl`, `--2xl` et `--3xl` descendent d'un cran dans un
 * `@media`. Le `<style>` du thème est émis APRÈS la feuille, à spécificité égale
 * — donc un `:root` de thème aurait GAGNÉ partout, mobile compris, et supprimé
 * cette réduction en silence. La sonde vérifie les deux moitiés : que `style.css`
 * déclare bien ce palier (sinon le piège n'existe pas et la précaution est du
 * bruit), et que le thème le réémet.
 *
 * ✅ **Elle remet tout en place** : brouillon capturé et restauré, image de test
 * supprimée, médiathèque rendue à son compte de départ.
 */
#[AsCommand(name: 'app:s166:presets-probe', description: 'S166 : prouve que le préréglage livré n\'émet RIEN, que la densité réémet le palier mobile, qu\'un préréglage inconnu est refusé, et qu\'une icône dérivée est écrite puis supprimée.')]
final class S166PresetsProbeCommand extends Command
{
    public function __construct(
        private readonly ThemePresets $presets,
        private readonly ThemeManager $themes,
        private readonly SiteMediaLibrary $media,
        private readonly string $projectDir,
        private readonly string $uploadDir,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];

        $io->section('1. 🔴 Le préréglage LIVRÉ n\'émet RIEN');
        // Sans choix, aucune règle : le balisage reste identique au bit près.
        // C'est la même garantie qu'à S160 pour les e-mails, et c'est ce qui rend
        // « je n'ai rien changé » vérifiable plutôt que promis.
        $this->check($io, $failures, 'trois axes au standard ⇒ chaîne vide', $this->presets->css('standard', 'standard', 'standard') === '');
        $this->check($io, $failures, 'et des valeurs vides aussi', $this->presets->css('', '', '') === '');

        $io->section('2. 🔴 La densité réémet le PALIER MOBILE');
        $sheet = (string) @file_get_contents($this->projectDir . '/public/css/style.css');
        // ⚠️ La moitié qu'on oublie de vérifier : si `style.css` cessait de
        // déclarer ce palier, la précaution ci-dessous deviendrait du bruit, et
        // personne ne le saurait.
        $this->check($io, $failures, '`style.css` déclare bien un palier sous 576 px', preg_match('/@media \(max-width: 576px\)\s*\{\s*:root\s*\{[^}]*--spacing-lg/', $sheet) === 1);

        $css = $this->presets->css('standard', 'aeree', 'standard');
        $this->check($io, $failures, 'la densité émet le barème de base', str_contains($css, '--spacing-lg: 30px'));
        $this->check($io, $failures, '🔴 ET le palier mobile, dans le même `@media`', str_contains($css, '@media (max-width: 576px)') && str_contains($css, '--spacing-lg: 20px'));
        $this->check($io, $failures, 'le rayon, lui, n\'a rien émis', !str_contains($css, '--border-radius'));

        $io->section('3. `--font-size-md` n\'est PAS figé');
        // `style.css` le définit comme `var(--font-size-base)` : il suit tout
        // seul. L'émettre en pixels le figerait au premier changement de barème.
        $typed = $this->presets->css('standard', 'standard', 'grande');
        $this->check($io, $failures, 'le barème typographique bouge', str_contains($typed, '--font-size-base: 18px'));
        $this->check($io, $failures, '⚠️ `--font-size-md` n\'est pas réécrit', !str_contains($typed, '--font-size-md'));

        $io->section('4. Un préréglage inconnu est REFUSÉ au point de passage');
        $before = $this->themes->draft();
        $refused = null;
        try {
            // 🔴 `array_merge`, PAS `$before + [...]` : l'union de tableaux GARDE
            // la valeur de gauche quand la clé existe des deux côtés — le
            // brouillon a déjà `radius`, donc la valeur hostile n'entrait jamais
            // et la sonde se mesurait elle-même. Trouvé en la voyant échouer.
            $this->themes->saveDraft(array_merge($before, ['radius' => 'gelule']));
        } catch (\InvalidArgumentException $e) {
            $refused = $e->getMessage();
        } finally {
            $this->themes->saveDraft($before);
        }
        $this->check($io, $failures, '🔴 `saveDraft()` refuse « gelule »', $refused !== null);
        if ($refused !== null) {
            $io->writeln('   « ' . $refused . ' »');
            // ⚠️ Le message NOMME la valeur : « préréglage inconnu » tout court
            // enverrait chercher lequel des trois axes est en cause.
            $this->check($io, $failures, 'et le message nomme la valeur', str_contains($refused, 'gelule'));
        }
        $this->check($io, $failures, 'et le brouillon n\'a pas bougé', $this->themes->draft() == $before);

        $io->section('5. L\'icône dérivée est écrite, carrée, et repartie avec l\'image');
        $tmp = sys_get_temp_dir() . '/s166-' . bin2hex(random_bytes(4));
        @mkdir($tmp, 0775, true);
        $mediaBefore = \count($this->media->all());
        $id = null;

        try {
            $png = $tmp . '/source.png';
            $this->writeWideTransparentPng($png, 400, 120);
            $stored = $this->media->store(new UploadedFile($png, 'logo.png', 'image/png', null, true));
            $this->check($io, $failures, 'l\'image entre', $stored['ok']);

            if ($stored['ok'] ?? false) {
                $id = (string) $stored['mediaId'];
                $icon = $this->uploadDir . '/' . $id . '-icon.png';
                $this->check($io, $failures, 'une dérivée est écrite', is_file($icon));

                if (is_file($icon)) {
                    $size = @getimagesize($icon);
                    // ⚠️ CARRÉE et centrée : un logo large étiré en 64×64
                    // deviendrait illisible.
                    $this->check($io, $failures, 'elle est CARRÉE', ($size[0] ?? 0) === 64 && ($size[1] ?? 0) === 64);
                    // 🔴 Transparence conservée : l'aplatir sur du blanc collerait
                    // un carré blanc dans un onglet sombre.
                    $this->check($io, $failures, '🔴 et sa transparence est conservée', in_array(ord(@file_get_contents($icon, false, null, 25, 1) ?: "\0"), [4, 6], true));
                    $this->check($io, $failures, '`iconPath()` rend la dérivée, pas l\'originale', str_ends_with((string) $this->media->iconPath($id), '-icon.png'));
                }

                $this->media->delete($id, []);
                $id = null;
                $this->check($io, $failures, 'la dérivée part avec l\'image', !is_file($icon));
            }
        } finally {
            if ($id !== null) {
                $this->media->delete($id, []);
            }
            array_map('unlink', glob($tmp . '/*') ?: []);
            @rmdir($tmp);
        }

        $this->check($io, $failures, 'la médiathèque est rendue à son compte de départ', \count($this->media->all()) === $mediaBefore);

        $io->section('6. Le logo SOMBRE est protégé comme les autres');
        // Supprimer l'image qui sert de logo sombre laisserait une image cassée
        // dans l'en-tête, visible seulement en thème sombre — donc découverte
        // tard.
        $this->check($io, $failures, 'le brouillon porte un champ `logoDarkPath`', array_key_exists('logoDarkPath', $this->themes->draft()));
        $io->writeln('   références du thème : ' . (implode(', ', $this->themes->referencedMediaIds()) ?: '(aucune)'));

        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }

        $io->success('Sonde S166 (préréglages) verte. Brouillon et médiathèque remis en place.');

        return Command::SUCCESS;
    }

    /** Large et TRANSPARENTE : les deux propriétés que la dérivée doit préserver. */
    private function writeWideTransparentPng(string $path, int $width, int $height): void
    {
        $image = imagecreatetruecolor($width, $height);
        imagealphablending($image, false);
        imagesavealpha($image, true);
        imagefilledrectangle($image, 0, 0, $width, $height, (int) imagecolorallocatealpha($image, 0, 0, 0, 127));
        imagealphablending($image, true);
        imagefilledrectangle($image, 10, 10, $width - 10, $height - 10, (int) imagecolorallocate($image, 158, 27, 86));
        imagealphablending($image, false);
        imagesavealpha($image, true);
        imagepng($image, $path);
        imagedestroy($image);
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
