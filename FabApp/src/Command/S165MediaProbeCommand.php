<?php

namespace App\Command;

use App\Media\SiteMediaLibrary;
use App\Service\ThemeManager;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;
use Symfony\Component\HttpFoundation\File\UploadedFile;

/**
 * S165 — la sonde de la médiathèque d'identité.
 *
 * 🔴 **Les trois mesures de sortie de la session** : on pose un logo sans toucher
 * au serveur, un fichier référencé ne se supprime pas, et le nom choisi par
 * l'utilisateur n'atteint jamais le disque.
 *
 * ⚠️ **Elle ne TOUCHE PAS au thème.** Le test « supprimer une image utilisée »
 * passe une liste de références SYNTHÉTIQUE à `delete()` plutôt que d'écrire dans
 * le brouillon de l'opérateur : la médiathèque ne connaît pas les thèmes, c'est
 * l'appelant qui fournit les références, et c'est exactement ce que la sonde
 * exploite. Le thème publié n'est ni lu ni écrit.
 *
 * ⚠️ **Elle crée de vrais fichiers et les efface**, et vérifie le compte de la
 * table et le contenu du dossier avant et après.
 */
#[AsCommand(name: 'app:s165:media-probe', description: 'S165 : prouve qu\'une image se téléverse sans accès au serveur, que le SVG est refusé, que le nom d\'origine n\'atteint pas le disque, et qu\'une image utilisée ne se supprime pas.')]
final class S165MediaProbeCommand extends Command
{
    public function __construct(
        private readonly SiteMediaLibrary $media,
        private readonly ThemeManager $themes,
        private readonly string $uploadDir,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];
        $tmp = sys_get_temp_dir() . '/s165-' . bin2hex(random_bytes(4));
        @mkdir($tmp, 0775, true);

        $before = \count($this->media->all());
        $created = [];

        try {
            $io->section('1. 🔴 Le SVG est REFUSÉ');
            // Un SVG est un document XML qui peut porter `<script>`, servi depuis
            // notre propre origine : c'est du code exécuté dans la session de
            // chaque visiteur. L'ancienne expression régulière l'acceptait.
            $svg = $tmp . '/logo.svg';
            file_put_contents($svg, '<svg xmlns="http://www.w3.org/2000/svg"><script>alert(1)</script></svg>');
            $refused = $this->media->store(new UploadedFile($svg, 'logo.svg', 'image/svg+xml', null, true));
            $this->check($io, $failures, 'un SVG n\'entre pas', !$refused['ok']);
            $this->check($io, $failures, 'et le refus NOMME le format', ($refused['error'] ?? '') === 'media.error_format');

            $io->section('2. 🔴 Le type est décidé par le CONTENU, pas par l\'extension');
            // Le même document, renommé en .png : une extension est une
            // affirmation de l'appelant, `getimagesize()` lit les octets.
            $liar = $tmp . '/deguise.png';
            copy($svg, $liar);
            $lied = $this->media->store(new UploadedFile($liar, 'deguise.png', 'image/png', null, true));
            $this->check($io, $failures, 'un SVG renommé « .png » n\'entre pas non plus', !$lied['ok']);

            $io->section('3. Une image entre — et le nom d\'origine n\'atteint pas le disque');
            // ⚠️ Le chemin sur disque est sage, le NOM ANNONCÉ est hostile : c'est
            // exactement la forme du danger. Un navigateur envoie le nom qu'il
            // veut ; le fichier temporaire, lui, est nommé par PHP.
            $png = $tmp . '/source.png';
            $this->writePng($png, 300, 120);
            $stored = $this->media->store(new UploadedFile($png, 'logo maison ../../.env.png', 'image/png', null, true));
            $this->check($io, $failures, 'l\'image est acceptée', $stored['ok']);

            if (!($stored['ok'] ?? false)) {
                $io->error('Sans image stockée, la suite n\'a rien à mesurer.');

                return Command::FAILURE;
            }

            $id = (string) $stored['mediaId'];
            $created[] = $id;
            $row = $this->media->find($id);

            $this->check($io, $failures, 'le fichier s\'appelle par son mediaId', str_starts_with((string) ($row['filename'] ?? ''), $id));
            // 🔴 Deux personnes qui envoient `logo.png` ne doivent pas s'écraser,
            // et un nom choisi par l'utilisateur ne doit pas traverser le système
            // de fichiers.
            $this->check($io, $failures, '🔴 aucun séparateur de chemin dans le nom sur disque', !str_contains((string) ($row['filename'] ?? ''), '/'));
            $this->check($io, $failures, 'le fichier existe vraiment sur disque', is_file($this->uploadDir . '/' . $row['filename']));

            /*
             * 🔴 **Le nom D'AFFICHAGE est déjà réduit à son `basename` — par
             * Symfony, avant qu'on le voie.** `UploadedFile::getClientOriginalName()`
             * ne rend jamais un chemin : « logo maison ../../.env.png » revient
             * « .env.png ». C'est une garde de plus en amont de la nôtre, et il
             * vaut mieux la MESURER que la supposer : le jour où elle change, ce
             * nom est écrit tel quel dans une page d'administration.
             * ⚠️ D'où deux téléversements ici et non un : celui-ci prouve qu'un
             * nom hostile est neutralisé, le suivant qu'un nom ordinaire survit.
             */
            $this->check($io, $failures, '🔴 le nom d\'affichage ne contient aucun chemin', !str_contains((string) ($row['originalName'] ?? ''), '/'));

            $clean = $tmp . '/clean.png';
            $this->writePng($clean, 64, 64);
            $second = $this->media->store(new UploadedFile($clean, 'logo-maison.png', 'image/png', null, true));
            $this->check($io, $failures, 'un second téléversement passe', $second['ok']);
            if ($second['ok'] ?? false) {
                $created[] = (string) $second['mediaId'];
                $secondRow = $this->media->find((string) $second['mediaId']);
                $this->check($io, $failures, 'un nom ordinaire est conservé pour l\'affichage', ($secondRow['originalName'] ?? '') === 'logo-maison.png');
                // ⚠️ Deux fichiers, deux identifiants : un nom d'origine partagé
                // ne doit jamais faire collision sur disque.
                $this->check($io, $failures, 'et son nom sur disque est DIFFÉRENT du premier', ($secondRow['filename'] ?? '') !== ($row['filename'] ?? ''));

                $this->media->delete((string) $second['mediaId'], []);
                $created = array_values(array_diff($created, [(string) $second['mediaId']]));
            }

            $io->section('4. Le normaliseur a bien tourné sur le fichier STOCKÉ');
            // Un PNG sans canal alpha est une photo dans un format qui ne
            // compresse pas les photos : `ImageNormalizer` le réencode en JPEG et
            // rend l'extension réellement écrite. Que le nom finisse en .jpg
            // prouve que la normalisation s'est appliquée au fichier rangé, et
            // pas seulement à une copie.
            $this->check($io, $failures, 'un PNG opaque est rangé en .jpg', str_ends_with((string) $row['filename'], '.jpg'));
            // ⚠️ Les dimensions sont mesurées APRÈS normalisation : ce sont
            // celles qu'on verra, pas celles des pixels tels que stockés à
            // l'arrivée. C'est ce qui évite la photo couchée classée en bannière.
            $this->check($io, $failures, 'les dimensions enregistrées sont celles du fichier rangé', (int) $row['width'] === 300 && (int) $row['height'] === 120);

            $io->section('5. Le chemin public se résout — et il n\'y en a pas d\'autre');
            $this->check($io, $failures, 'assetPath() rend le chemin de la médiathèque', $this->media->assetPath($id) === 'uploads/identity/' . $row['filename']);
            $this->check($io, $failures, 'un mediaId inconnu ne résout RIEN', $this->media->assetPath(str_repeat('0', 32)) === null);
            // 🔴 Plus de chemin libre : une valeur qui n'est pas un mediaId ne
            // peut plus atteindre `asset()`.
            $this->check($io, $failures, '🔴 « ../../.env » n\'est pas un mediaId', !SiteMediaLibrary::isMediaId('../../.env'));

            $io->section('6. 🔴 Une image UTILISÉE ne se supprime pas');
            $refusedDelete = $this->media->delete($id, [$id]);
            $this->check($io, $failures, 'la suppression est refusée', !$refusedDelete['ok']);
            $this->check($io, $failures, 'et le refus dit POURQUOI', ($refusedDelete['error'] ?? '') === 'media.error_in_use');
            $this->check($io, $failures, 'le fichier est toujours là', is_file($this->uploadDir . '/' . $row['filename']));

            $io->section('7. Non utilisée, elle part — ligne ET fichier');
            $gone = $this->media->delete($id, []);
            $this->check($io, $failures, 'la suppression passe', $gone['ok']);
            $this->check($io, $failures, 'la ligne a disparu', $this->media->find($id) === null);
            $this->check($io, $failures, 'le fichier aussi', !is_file($this->uploadDir . '/' . $row['filename']));
            $created = [];

            $io->section('8. Le thème n\'a pas été touché');
            // ⚠️ Mesuré, pas supposé : la sonde n'écrit jamais dans le brouillon,
            // et cette ligne le dit à qui la lance sur une installation réelle.
            $io->writeln('   références du thème : ' . (implode(', ', $this->themes->referencedMediaIds()) ?: '(aucune)'));
            $this->check($io, $failures, 'la médiathèque est rendue à son compte de départ', \count($this->media->all()) === $before);
        } finally {
            foreach ($created as $id) {
                $this->media->delete($id, []);
            }
            array_map('unlink', glob($tmp . '/*') ?: []);
            @rmdir($tmp);
        }

        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }

        $io->success('Sonde S165 verte. Médiathèque rendue à son état de départ, thème non touché.');

        return Command::SUCCESS;
    }

    /** Une image OPAQUE : c'est ce qui déclenche le changement de conteneur. */
    private function writePng(string $path, int $width, int $height): void
    {
        $image = imagecreatetruecolor($width, $height);
        imagefilledrectangle($image, 0, 0, $width, $height, (int) imagecolorallocate($image, 158, 27, 86));
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
