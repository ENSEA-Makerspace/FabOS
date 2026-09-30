<?php

namespace App\Command;

use App\Entity\Badge;
use App\Entity\Event;
use App\Entity\EventCategory;
use App\Entity\Formation;
use App\Entity\Loan;
use App\Entity\Place;
use App\Entity\Reservation;
use App\Entity\Utilisateur;
use App\Reservation\ReservableType;
use App\Entity\LoanableItem;
use App\Entity\Machine;
use App\Entity\MachineBadge;
use App\Entity\MachineCategory;
use App\Entity\Material;
use App\Entity\Venue;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Input\InputOption;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;

/**
 * Données de démonstration : le VRAI parc du fablab de l'opérateur (2026-09-27).
 *
 * Epilog Fusion Pro 48, xTool F1 Ultra, 12 Bambu Lab X1C (PLA seulement),
 * Roland VersaSTUDIO BN-20, Ricoma EM-1010, deux Sawgrass et une presse à chaud,
 * un four à refusion CIF, 4 postes de soudure ; MDF et contreplaqué de peuplier
 * pour les lasers ; une caméra thermique en prêt. La salle de cours existe déjà.
 *
 * ✅ Idempotente : chaque objet est cherché par son nom avant d'être créé ;
 * relancer ne duplique rien. `--dry-run` montre le plan et n'écrit rien.
 * ⚠️ Trois machines de dev correspondent à du vrai matériel et gardent leur
 * historique (badges, quiz, réservations) : elles sont RENOMMÉES, pas recréées.
 * ⚠️ Les machines de dev absentes du labo sont ARCHIVÉES (décision de
 * l'opérateur) — jamais supprimées : réservations et journaux les nomment.
 * ⚠️ Aucun modèle ni aucune puissance devinés : seulement ce que l'opérateur a
 * dit, et les caractéristiques publiques des modèles nommés.
 */
#[AsCommand(name: 'app:seed:lab-demo', description: 'Données de démo : le parc réel du fablab (machines, matières, prêt). Idempotent ; --dry-run.')]
final class SeedLabDemoCommand extends Command
{
    /** Machines de dev renommées : ancien nom → nouveau nom. */
    private const RENAME = [
        'Découpeuse Laser CO2' => 'Epilog Fusion Pro 48',
        'Brodeuse Numérique' => 'Ricoma EM-1010',
        'Station de Soudure Électronique' => 'Station de soudure n°1',
    ];

    /** Machines de dev absentes du labo : archivées. */
    private const ARCHIVE = [
        'Imprimante 3D test', 'The Beast', 'Imprimante 3D Prusa i3 MK3S', 'Découpeuse Vinyle Silhouette',
        'Fraiseuse CNC', 'Oscilloscope Numérique', 'Uranus',
    ];

    private const CATEGORIES = [
        'decoupe' => 'Découpe', 'impression-3d' => 'Impression 3D', 'textile' => 'Textile', 'electronique' => 'Électronique',
    ];

    /** @var list<string> */
    private array $plan = [];

    /**
     * ⚠️ Les renommées, par leur NOUVEAU nom : avant le flush, la base porte encore
     * l'ancien, et une recherche par nom recréerait la machine en double.
     *
     * @var array<string, Machine>
     */
    private array $renamed = [];

    public function __construct(private readonly EntityManagerInterface $em, private readonly \App\Service\PlaceBadges $placeBadges, private readonly \App\Training\BadgeGrants $grants)
    {
        parent::__construct();
    }

    protected function configure(): void
    {
        $this->addOption('dry-run', null, InputOption::VALUE_NONE, 'Montrer le plan, ne rien écrire.');
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $dry = (bool) $input->getOption('dry-run');

        $venue = $this->em->getRepository(Venue::class)->findOneBy(['slug' => 'default']);
        if (!$venue instanceof Venue) {
            $io->error('Le lieu « default » (FabLab) est introuvable.');

            return Command::FAILURE;
        }
        $badge = fn (string $name): ?Badge => $this->em->getRepository(Badge::class)->findOneBy(['nom' => $name]);
        [$laser, $maker3d, $soudure, $vinyle, $brodeuse] = [
            $badge('Découpe Laser - Niveau 1'), $badge('Maker 3D'), $badge('Badge Soudure Électronique'),
            $badge('Badge Découpe Vinyle'), $badge('Badge Brodeuse Numérique'),
        ];

        foreach (self::CATEGORIES as $label) {
            if (!$this->em->getRepository(MachineCategory::class)->findOneBy(['label' => $label])) {
                $this->note("catégorie créée : $label");
                $this->em->persist((new MachineCategory())->setLabel($label));
            }
        }
        foreach (self::RENAME as $old => $new) {
            $machine = $this->machine($old);
            if ($machine instanceof Machine && !$this->machine($new)) {
                $this->note("renommée : $old → $new (historique gardé)");
                $machine->setNom($new);
                $this->renamed[$new] = $machine;
            }
        }
        foreach (self::ARCHIVE as $name) {
            $machine = $this->machine($name);
            if ($machine instanceof Machine && $machine->getArchivedAt() === null) {
                $this->note("archivée : $name");
                $machine->archive();
            }
        }

        $x1c = 'Imprimante FDM, volume 256 × 256 × 256 mm, plateau chauffant, caméra de suivi.';
        $machines = [
            ['Epilog Fusion Pro 48', 'Epilog', 'Fusion Pro 48', 'decoupe', 'Laser CO2 80 W grand format, plateau 1219 × 914 mm : découpe et gravure du bois, du MDF, du contreplaqué, de l’acrylique, du cuir, du carton.', [$laser]],
            ['xTool F1 Ultra', 'xTool', 'F1 Ultra', 'decoupe', 'Laser portable à double source — fibre 20 W (métal, gravure fine) et diode 20 W (bois, cuir) — pour la gravure de petits objets et de petites séries.', [$laser]],
            ['Roland VersaSTUDIO BN-20', 'Roland DG', 'VersaSTUDIO BN-20', 'decoupe', 'Imprimante-découpeuse éco-solvant, laize 515 mm : autocollants, étiquettes, flex textile imprimé.', [$vinyle]],
            ['Ricoma EM-1010', 'Ricoma', 'EM-1010', 'textile', 'Brodeuse 10 aiguilles, une tête : écussons, t-shirts, sacs, casquettes.', [$brodeuse]],
            ['Sawgrass n°1', 'Sawgrass', 'SG1000', 'textile', 'Imprimante à sublimation A3 : visuels transférés ensuite à la presse à chaud sur textile polyester, mugs, tapis de souris.', []],
            ['Sawgrass n°2', 'Sawgrass', 'SG1000', 'textile', 'Imprimante à sublimation A3 : visuels transférés ensuite à la presse à chaud sur textile polyester, mugs, tapis de souris.', []],
            ['Presse à chaud', null, null, 'textile', 'Transfert des impressions sublimation et du flex sur textile et objets plats.', []],
            ['Four à refusion CIF 03', 'CIF', null, 'electronique', 'Four à refusion pour souder des cartes électroniques CMS après dépose de pâte à braser.', [$soudure]],
        ];
        for ($n = 1; $n <= 4; ++$n) {
            $machines[] = ["Station de soudure n°$n", null, null, 'electronique', 'Poste de soudure équipé pour l’électronique traversante et CMS.', [$soudure]];
        }
        for ($n = 1; $n <= 12; ++$n) {
            $machines[] = [sprintf('Bambu Lab X1C n°%02d', $n), 'Bambu Lab', 'X1 Carbon', 'impression-3d', $x1c, [$maker3d]];
        }

        $byName = [];
        foreach ($machines as [$name, $maker, $model, $slug, $description, $badges]) {
            $machine = $this->machine($name);
            if (!$machine instanceof Machine) {
                $this->note("machine créée : $name");
                $machine = (new Machine())->setNom($name)->setVenue($venue)->setMachineToken(bin2hex(random_bytes(8)))->setStatut('disponible');
                $this->em->persist($machine);
            }
            $machine->setManufacturer($maker)->setModel($model)->setDescription($description)
                ->setCategorySlug($slug)->setCategoryLabel(self::CATEGORIES[$slug])->setIconSlug($slug);
            if ($slug === 'impression-3d') {
                $machine->setRequirementTitle('PLA uniquement')
                    ->setRequirementDescription('Au fablab, on imprime seulement du PLA ; les autres matières s’impriment ailleurs.');
            }
            foreach (array_filter($badges) as $b) {
                $linked = $machine->getId() !== null && $this->em->getRepository(MachineBadge::class)->findOneBy(['machine' => $machine, 'badge' => $b]);
                if (!$linked) {
                    $this->note(sprintf('  badge exigé : %s → %s', $b->getNom(), $name));
                    $this->em->persist((new MachineBadge())->setMachine($machine)->setBadge($b)->setRequiredForAccess(true));
                }
            }
            $byName[$name] = $machine;
        }

        $lasers = [$byName['Epilog Fusion Pro 48'], $byName['xTool F1 Ultra']];
        $printers = array_values(array_filter($byName, static fn (Machine $m): bool => str_starts_with($m->getNom(), 'Bambu Lab')));
        foreach ([
            ['PLA', 'Filament', 'Filament PLA 1,75 mm — la seule matière imprimée au fablab.', $printers],
            ['MDF', 'Panneau', 'Panneau de fibres pour la découpe et la gravure laser.', $lasers],
            ['Contreplaqué de peuplier', 'Panneau', 'Contreplaqué léger pour la découpe et la gravure laser.', $lasers],
        ] as [$name, $category, $description, $users]) {
            $material = $this->em->getRepository(Material::class)->findOneBy(['name' => $name]);
            if (!$material instanceof Material) {
                $this->note("matière créée : $name");
                $material = (new Material())->setName($name)->setCategory($category);
                $this->em->persist($material);
            }
            $material->setDescription($material->getDescription() ?: $description);
            foreach ($users as $machine) {
                if (!$material->getMachines()->contains($machine)) {
                    $material->getMachines()->add($machine);
                }
            }
        }

        if (!$this->em->getRepository(LoanableItem::class)->findOneBy(['name' => 'Caméra thermique'])) {
            $this->note('prêt créé : Caméra thermique (1)');
            $this->em->persist((new LoanableItem())->setName('Caméra thermique')->setCategory('Mesure')->setQuantity(1)->setVenue($venue)
                ->setDescription('Caméra thermique à emprunter : points chauds d’une carte électronique, fuites thermiques, contrôle d’une pièce.'));
        }

        $this->formations();
        $this->factory();
        $this->photos();
        $this->events($venue);
        $this->activity($byName, $venue);

        foreach ($this->plan as $line) {
            $io->writeln('   ' . $line);
        }
        if ($dry) {
            $io->note(\count($this->plan) . ' changement(s) prévus — rien n’a été écrit (--dry-run).');

            return Command::SUCCESS;
        }
        $this->em->flush();
        $before = \count($this->plan);
        $this->factoryAccess();
        $this->demoBadges();
        foreach (\array_slice($this->plan, $before) as $line) {
            $io->writeln('   ' . $line);
        }
        $this->em->flush();
        $io->success(\count($this->plan) . ' changement(s) écrits.');

        return Command::SUCCESS;
    }

    /**
     * Les formations suivent le parc : celle de la découpe vinyle devient celle
     * de la Roland ; celles de la CNC et de l'oscilloscope (archivés) sont
     * archivées avec leur badge ; la sublimation gagne une formation.
     */
    private function formations(): void
    {
        $formations = $this->em->getRepository(Formation::class);
        $vinyle = $formations->findOneBy(['titre' => 'Formation découpe vinyle']);
        if ($vinyle instanceof Formation) {
            $this->note('formation renommée : Formation découpe vinyle → Formation Roland BN-20 : impression et découpe');
            $vinyle->setTitre('Formation Roland BN-20 : impression et découpe');
        }
        foreach ($formations->findAll() as $formation) {
            $title = mb_strtolower($formation->getTitre());
            if ($formation->getArchivedAt() === null && (str_contains($title, 'fraiseuse cnc') || str_contains($title, 'oscilloscope'))) {
                $this->note('formation archivée : ' . $formation->getTitre());
                $formation->archive();
            }
        }
        foreach (['Badge CNC', 'Badge Oscilloscope'] as $name) {
            $badge = $this->em->getRepository(Badge::class)->findOneBy(['nom' => $name]);
            if ($badge instanceof Badge && $badge->getArchivedAt() === null) {
                $this->note("badge archivé : $name");
                $badge->archive();
            }
        }
        if (!$formations->findOneBy(['titre' => 'Sublimation et presse à chaud'])) {
            $this->note('formation créée : Sublimation et presse à chaud');
            $this->em->persist((new Formation())->setTitre('Sublimation et presse à chaud')->setNiveau(1)->setDuree('1 h 30')
                ->setCategorie('Textile')->setPlacesTotales(8)
                ->setDescription('Préparer un visuel, l’imprimer sur une Sawgrass SG1000 et le transférer à la presse à chaud sur un t-shirt polyester ou un mug.')
                ->setObjectifs('Choisir le bon support ; régler temps, température et pression ; éviter les dédoublements et les brûlures.')
                ->setMaterielFourni('Encre et papier sublimation, un t-shirt polyester par personne.'));
        }
    }

    /**
     * La Factory (2026-09-30) : un second lieu, trois pièces réservables —
     * MetalFab, WoodFab, PrintLab —, chacune avec sa porte, sa formation d'accès
     * et son badge. Toutes les machines d'une pièce exigent le badge de la pièce.
     *
     * ⚠️ Une PIÈCE n'exige pas encore de badge (ni pour la réserver, ni à sa
     * porte, qui s'ouvre sur une réservation en cours) : c'est une fonctionnalité
     * à construire. Ici, seules les machines sont gardées par le badge.
     * ⚠️ La CNC 4x8 existait (archivée, avec son historique) : elle est
     * RÉACTIVÉE et déplacée, pas recréée.
     */
    private function factory(): void
    {
        $venues = $this->em->getRepository(Venue::class);
        $factory = $venues->findOneBy(['slug' => 'factory']);
        if (!$factory instanceof Venue) {
            $this->note('lieu créé : Factory');
            $factory = (new Venue())->setSlug('factory')->setName('Factory')->setTimezone('Europe/Paris');
            $this->em->persist($factory);
        }
        foreach (['Métal' => 'metal', 'Usinage' => 'usinage', 'Thermoformage' => 'thermoformage'] as $label => $icon) {
            $category = $this->em->getRepository(MachineCategory::class)->findOneBy(['label' => $label]);
            if (!$category instanceof MachineCategory) {
                $this->note("catégorie créée : $label");
                $this->em->persist((new MachineCategory())->setLabel($label));
            } elseif ($category->getArchivedAt() !== null) {
                $this->note("catégorie réactivée : $label");
                $category->restore();
            }
        }

        $rooms = [
            'MetalFab' => [4, 'Atelier métal : soudure et découpe laser du métal.', 'Soudure et découpe laser du métal : EPI, fumées, risques laser classe 4, zones interdites.'],
            'WoodFab' => [6, 'Atelier bois : usinage numérique grand format.', 'Usinage bois : bridage, fixation du martyr, poussières, arrêt d’urgence, choix des fraises.'],
            'PrintLab' => [8, 'Impression 3D technique (TPU, nylon, PETG, ASA) et thermoformage.', 'Matières techniques : températures, ventilation des vapeurs (ASA, nylon), séchage des filaments, thermoformeuse.'],
        ];
        $badges = [];
        foreach ($rooms as $room => [$capacity, $description, $training]) {
            $place = $this->em->getRepository(Place::class)->findOneBy(['nom' => $room, 'venue' => $factory->getId() ? $factory : null]);
            if (!$place instanceof Place) {
                $this->note("pièce réservable créée : $room (Factory)");
                $place = (new Place())->setNom($room)->setVenue($factory)->setCapacite($capacity)->setDescription($description);
                $this->em->persist($place);
            }
            if (!$this->em->getRepository(\App\Entity\AccessPoint::class)->findOneBy(['nom' => "Porte $room"])) {
                $this->note("porte créée : Porte $room");
                $this->em->persist((new \App\Entity\AccessPoint())->setNom("Porte $room")->setKind('door')->setVenue($factory)->setPlace($place)
                    ->setDescription("S’ouvre pendant une réservation de la pièce $room."));
            }
            $badge = $this->em->getRepository(Badge::class)->findOneBy(['nom' => "Accès $room"]);
            if (!$badge instanceof Badge) {
                $this->note("badge créé : Accès $room");
                $badge = (new Badge())->setNom("Accès $room")->setDescription("Autorise les machines de la pièce $room (Factory).");
                $this->em->persist($badge);
            }
            $badges[$room] = $badge;
            if (!$this->em->getRepository(Formation::class)->findOneBy(['titre' => "Formation d’accès $room"])) {
                $this->note("formation créée : Formation d’accès $room");
                $this->em->persist((new Formation())->setTitre("Formation d’accès $room")->setBadge($badge)->setNiveau(2)
                    ->setDuree('2 h')->setPlacesTotales(6)->setRequiresPractical(true)->setCategorie('Factory')
                    ->setDescription("Obligatoire pour utiliser les machines de la pièce $room. Théorie en ligne, puis validation en personne.")
                    ->setObjectifs($training));
            }
        }

        $cnc = $this->machine('CNC 4x8');
        if ($cnc instanceof Machine) {
            if ($cnc->getArchivedAt() !== null) {
                $this->note('machine réactivée et déplacée : CNC 4x8 → Factory / WoodFab (historique gardé)');
                $cnc->restore();
            }
            // Son ancien badge (« Badge CNC ») est archivé : l'exiger encore fermerait la machine à tous.
            foreach ($this->em->getRepository(MachineBadge::class)->findBy(['machine' => $cnc]) as $link) {
                if ($link->getBadge()?->getArchivedAt() !== null) {
                    $this->note('lien retiré : CNC 4x8 n’exige plus le badge archivé « ' . $link->getBadge()?->getNom() . ' »');
                    $this->em->remove($link);
                }
            }
        }

        $k1 = 'Imprimante FDM 300 × 300 × 300 mm, buse haute température, caisson fermé.';
        $machines = [
            ['xTool MetalFab 800', 'xTool', 'MetalFab', 'Métal', 'metal', 'MetalFab', 'Laser fibre 800 W : soudure et découpe de l’acier, de l’inox et de l’aluminium.', null],
            ['CNC 4x8', null, null, 'Usinage', 'usinage', 'WoodFab', 'Fraiseuse numérique grand format, plateau 2440 × 1220 mm (4 × 8 pieds) : panneaux, mobilier, gabarits.', null],
            ['Mayku Proformer', 'Mayku', 'Proformer', 'Thermoformage', 'thermoformage', 'PrintLab', 'Thermoformeuse grand format : moules, coques, emballages sur plaques plastiques.', null],
            ['Creality K1 Max n°1', 'Creality', 'K1 Max', 'Impression 3D', 'impression-3d', 'PrintLab', $k1, 'TPU'],
            ['Creality K1 Max n°2', 'Creality', 'K1 Max', 'Impression 3D', 'impression-3d', 'PrintLab', $k1, 'Nylon'],
            ['Creality K1 Max n°3', 'Creality', 'K1 Max', 'Impression 3D', 'impression-3d', 'PrintLab', $k1, 'PETG'],
            ['Creality K1 Max n°4', 'Creality', 'K1 Max', 'Impression 3D', 'impression-3d', 'PrintLab', $k1, 'ASA'],
        ];
        foreach ($machines as [$name, $maker, $model, $label, $slug, $room, $description, $material]) {
            $machine = $this->machine($name);
            if (!$machine instanceof Machine) {
                $this->note("machine créée : $name (Factory / $room)");
                $machine = (new Machine())->setNom($name)->setMachineToken(bin2hex(random_bytes(8)))->setStatut('disponible');
                $this->em->persist($machine);
            }
            $machine->setVenue($factory)->setLocalisation($room)->setManufacturer($maker ?? $machine->getManufacturer())
                ->setModel($model ?? $machine->getModel())->setDescription($description)
                ->setCategorySlug($slug)->setCategoryLabel($label)->setIconSlug($slug);
            if ($material !== null) {
                $machine->setRequirementTitle("Matière : $material")
                    ->setRequirementDescription("Cette imprimante est réservée au $material ; les autres matières ont leur propre K1 Max.");
            }
            $badge = $badges[$room];
            $linked = $machine->getId() !== null && $badge->getId() !== null
                && $this->em->getRepository(MachineBadge::class)->findOneBy(['machine' => $machine, 'badge' => $badge]);
            if (!$linked) {
                $this->note("  badge exigé : Accès $room → $name");
                $this->em->persist((new MachineBadge())->setMachine($machine)->setBadge($badge)->setRequiredForAccess(true));
            }
            if ($material !== null) {
                $this->materialFor($material, $machine, 'Filament', "Filament $material pour l’impression technique (PrintLab, Factory).");
            }
        }
        $cncNow = $this->machine('CNC 4x8');
        foreach (['MDF', 'Contreplaqué de peuplier'] as $panel) {
            if ($cncNow instanceof Machine) {
                $this->materialFor($panel, $cncNow, 'Panneau', null);
            }
        }
    }

    /**
     * S204 — après l'écriture (il faut les identifiants) : chaque pièce de la
     * Factory exige son badge d'accès, et la Factory reprend les horaires du
     * FabLab — sans horaires, une porte « au badge seul » ne s'ouvrirait jamais.
     * ⚠️ Ne tourne pas en --dry-run (rien n'est encore en base à relier).
     */
    private function factoryAccess(): void
    {
        $factory = $this->em->getRepository(Venue::class)->findOneBy(['slug' => 'factory']);
        $fablab = $this->em->getRepository(Venue::class)->findOneBy(['slug' => 'default']);
        if (!$factory instanceof Venue || !$fablab instanceof Venue) {
            return;
        }
        if ($this->placeBadges->isReady()) {
            foreach (['MetalFab', 'WoodFab', 'PrintLab'] as $room) {
                $place = $this->em->getRepository(Place::class)->findOneBy(['nom' => $room, 'venue' => $factory]);
                $badge = $this->em->getRepository(Badge::class)->findOneBy(['nom' => "Accès $room"]);
                if ($place instanceof Place && $badge instanceof Badge && $this->placeBadges->requiredIds($place) === []) {
                    $this->note("pièce gardée : $room exige « Accès $room » (réservation, et porte au badge seul)");
                    $this->placeBadges->set($place, [$badge]);
                }
            }
        } else {
            $this->note('(migration S204 pas encore passée : pièces non encore gardées par badge)');
        }
        $hours = $this->em->getRepository(\App\Entity\OpeningHour::class);
        if ($hours->findBy(['venue' => $factory]) === []) {
            foreach ($hours->findBy(['venue' => $fablab, 'scopeType' => null]) as $row) {
                $this->em->persist((new \App\Entity\OpeningHour())->setVenue($factory)->setDayOfWeek($row->getDayOfWeek())
                    ->setLabel($row->getLabel())->setIsClosed($row->isClosed())->setOpenTime($row->getOpenTime())
                    ->setCloseTime($row->getCloseTime())->setSortOrder($row->getSortOrder()));
            }
            $this->note('horaires : la Factory reprend ceux du FabLab');
        }
    }

    /**
     * Les badges des comptes de TEST, par la voie officielle (S202 : journal
     * qui / quand / pourquoi, motif « Jeu de démonstration »), choisis pour que
     * les réservations de démo soient cohérentes et que chaque compte montre un
     * cas : bob n'en a aucun. ⚠️ Pas en --dry-run (BadgeGrants écrit tout de suite).
     */
    private function demoBadges(): void
    {
        if (!$this->grants->isReady()) {
            return;
        }
        $admin = null;
        foreach ($this->em->getRepository(Utilisateur::class)->findBy(['statut' => 'actif']) as $candidate) {
            if (\in_array('ROLE_ADMIN', $candidate->getRoles(), true)) {
                $admin = $candidate;
                break;
            }
        }
        if (!$admin instanceof Utilisateur) {
            return;
        }
        foreach ([
            'alice@example.org' => ['Maker 3D', 'Badge Brodeuse Numérique', 'Accès MetalFab'],
            'carol@example.org' => ['Découpe Laser - Niveau 1', 'Accès WoodFab'],
            'frank@example.org' => ['Maker 3D', 'Badge Soudure Électronique', 'Accès PrintLab'],
        ] as $email => $names) {
            $user = $this->em->getRepository(Utilisateur::class)->findOneBy(['email' => $email]);
            if (!$user instanceof Utilisateur) {
                continue;
            }
            foreach ($names as $name) {
                $badge = $this->em->getRepository(Badge::class)->findOneBy(['nom' => $name]);
                if ($badge instanceof Badge && $this->grants->grant($user, $badge, $admin, 'Jeu de démonstration')) {
                    $this->note("badge attribué : $name → $email");
                }
            }
        }
    }

    /**
     * Photos de démo : seulement des images SOUS LICENCE LIBRE (Wikimedia
     * Commons, crédits dans public/images/machines/demo/CREDITS.md), et
     * seulement celles qui montrent vraiment la machine — trois candidates
     * (CNC, four, thermoformeuse) ont été écartées à l'œil. Le crédit est
     * ajouté à la description, comme la licence l'exige. Une photo déjà posée
     * par l'exploitant n'est jamais remplacée.
     */
    private function photos(): void
    {
        $sets = [
            'Bambu Lab X1C' => ['bambu-x1c', 'Benlisquare, CC BY-SA 4.0'],
            'Creality K1 Max' => ['creality-k1', 'Mortymore, CC BY 2.0 (Creality K1C, même famille)'],
            'Presse à chaud' => ['presse-a-chaud', 'Scrud123, CC BY-SA 3.0'],
            'Station de soudure' => ['station-soudure', 'Leisto, CC BY-SA 4.0'],
        ];
        foreach ($this->em->getRepository(Machine::class)->findBy(['archivedAt' => null]) as $machine) {
            foreach ($sets as $prefix => [$file, $credit]) {
                if (!str_starts_with($machine->getNom(), $prefix) || ($machine->getPhoto() ?? '') !== '') {
                    continue;
                }
                $this->note(sprintf('photo : %s ← %s.jpg', $machine->getNom(), $file));
                $machine->setPhoto("images/machines/demo/$file.jpg");
                $line = "Photo : $credit, Wikimedia Commons.";
                if (!str_contains((string) $machine->getDescription(), $line)) {
                    $machine->setDescription(trim((string) $machine->getDescription() . "\n\n" . $line));
                }
            }
        }
    }

    private function materialFor(string $name, Machine $machine, string $category, ?string $description): void
    {
        $material = $this->em->getRepository(Material::class)->findOneBy(['name' => $name]);
        if (!$material instanceof Material) {
            $this->note("matière créée : $name");
            $material = (new Material())->setName($name)->setCategory($category)->setDescription($description);
            $this->em->persist($material);
        }
        if (!$material->getMachines()->contains($machine)) {
            $material->getMachines()->add($machine);
        }
    }

    /** Six rendez-vous à venir (heures du labo : saisies telles quelles, convention B). */
    private function events(Venue $venue): void
    {
        $category = fn (string $label): ?EventCategory => $this->em->getRepository(EventCategory::class)->findOneBy(['label' => $label]);
        $laserTraining = $this->em->getRepository(Formation::class)->findOneBy(['titre' => 'Formation découpe laser CO2']);
        $room = 'Salle de cours - Fablab';
        foreach ([
            ['Atelier découverte : imprimer en 3D avec les Bambu X1C', '2026-10-07 18:00', '2026-10-07 20:00', 'Atelier', 12, $room, 'Du fichier à l’objet : trancher un modèle, lancer une impression PLA et récupérer la pièce. Aucune expérience demandée.', null],
            ['Portes ouvertes du fablab', '2026-10-10 14:00', '2026-10-10 17:00', 'Portes ouvertes', null, 'Fablab', 'Démonstrations en continu : laser, impression 3D, broderie, sublimation. Entrée libre.', null],
            ['Atelier sublimation : personnalise ton t-shirt', '2026-10-14 18:00', '2026-10-14 20:30', 'Atelier', 8, 'Fablab', 'Un visuel, une Sawgrass, la presse à chaud : chacun repart avec son t-shirt.', null],
            ['Soirée électronique : un kit CMS au four à refusion', '2026-10-21 18:00', '2026-10-21 21:00', 'Atelier', 8, 'Fablab', 'Poser la pâte à braser, placer les composants, passer la carte au four CIF, puis retoucher au poste de soudure.', null],
            ['Séance de formation : découpe laser sur l’Epilog', '2026-10-28 18:00', '2026-10-28 20:00', 'Séance de formation', 6, 'Fablab', 'Partie pratique de la formation laser : réglages, focale, sécurité, première découpe en MDF.', $laserTraining],
            ['Initiation broderie numérique sur la Ricoma', '2026-11-04 18:00', '2026-11-04 20:00', 'Atelier', 6, 'Fablab', 'Préparer un motif, choisir les fils, lancer la brodeuse 10 aiguilles.', null],
        ] as [$title, $start, $end, $cat, $capacity, $where, $description, $formation]) {
            if ($this->em->getRepository(Event::class)->findOneBy(['titre' => $title])) {
                continue;
            }
            $this->note("événement créé : $title ($start)");
            $this->em->persist((new Event())->setTitre($title)->setDescription($description)
                ->setDateDebut(new \DateTimeImmutable($start))->setDateFin(new \DateTimeImmutable($end))
                ->setLieu($where)->setCapacite($capacity)->setVenue($venue)->setCategory($category($cat))->setFormation($formation));
        }
    }

    /**
     * Un peu de vie sur le tableau de bord : des réservations dans les jours qui
     * viennent et un prêt en cours. ⚠️ Au nom des comptes de TEST en
     * @example.org seulement (jamais bob@bobby.com, un vrai domaine) : un rappel
     * ou une annulation ne peut toucher personne. Rien ne passe par les services
     * de réservation : aucun courrier n'est envoyé à la création.
     *
     * @param array<string, Machine> $machines
     */
    private function activity(array $machines, Venue $venue): void
    {
        $user = fn (string $email): ?Utilisateur => $this->em->getRepository(Utilisateur::class)->findOneBy(['email' => $email]);
        [$alice, $carol, $frank] = [$user('alice@example.org'), $user('carol@example.org'), $user('frank@example.org')];
        if (!$alice || !$carol || !$frank) {
            $this->note('activité : comptes de test alice/carol/frank absents — réservations et prêt non créés');

            return;
        }
        $room = $this->em->getRepository(Place::class)->findOneBy(['nom' => 'Salle de cours - Fablab']);
        $bookings = [
            [$alice, $machines['Bambu Lab X1C n°03'], '2026-10-01 10:00', '2026-10-01 13:00', 'Boîtier pour capteur'],
            [$frank, $machines['Bambu Lab X1C n°07'], '2026-10-01 14:00', '2026-10-01 18:00', 'Pièces de robot'],
            [$carol, $machines['Epilog Fusion Pro 48'], '2026-10-02 09:00', '2026-10-02 10:30', 'Boîtes en contreplaqué peuplier'],
            [$alice, $machines['Ricoma EM-1010'], '2026-10-05 14:00', '2026-10-05 16:00', 'Écussons du club'],
            [$frank, $machines['Station de soudure n°2'], '2026-10-06 16:00', '2026-10-06 18:00', 'Carte capteur'],
        ];
        foreach ($bookings as [$who, $machine, $start, $end, $why]) {
            $this->book($who, ReservableType::Machine, (int) $machine->getId(), $machine->getNom(), $start, $end, $why);
        }
        if ($room instanceof Place) {
            $this->book($carol, ReservableType::Place, (int) $room->getId(), $room->getNom(), '2026-10-06 08:00', '2026-10-06 12:00', 'Cours de CAO');
        }

        $camera = $this->em->getRepository(LoanableItem::class)->findOneBy(['name' => 'Caméra thermique']);
        if ($camera instanceof LoanableItem && $camera->getId() !== null
            && !$this->em->getRepository(Loan::class)->findOneBy(['item' => $camera, 'status' => Loan::STATUS_OUT])) {
            $this->note('prêt en cours : Caméra thermique → frank (retour prévu le 06/10)');
            $this->em->persist((new Loan())->setItem($camera)->setBorrower($frank)->setBorrowerName($frank->getDisplayName())
                ->setBorrowerEmail($frank->getEmail())->setDateTaken(new \DateTimeImmutable('2026-09-29 10:00'))
                ->setExpectedReturnDate(new \DateTimeImmutable('2026-10-06'))->setStatus(Loan::STATUS_OUT)
                ->setNotes('Diagnostic d’une alimentation qui chauffe.'));
        }
    }

    private function book(Utilisateur $who, ReservableType $type, int $id, string $label, string $start, string $end, string $why): void
    {
        $from = new \DateTimeImmutable($start);
        if ($id === 0 || $this->em->getRepository(Reservation::class)->findOneBy(['reservableType' => $type->value, 'reservableId' => $id, 'dateDebut' => $from])) {
            return;
        }
        $this->note(sprintf('réservation : %s — %s, %s', $label, $who->getFirstName(), $start));
        $this->em->persist((new Reservation())->setUtilisateur($who)->setReservable($type, $id, $label)
            ->setDateDebut($from)->setDateFin(new \DateTimeImmutable($end))->setMotif($why)->setStatut(Reservation::STATUS_CONFIRMED));
    }

    private function machine(string $name): ?Machine
    {
        return $this->renamed[$name] ?? $this->em->getRepository(Machine::class)->findOneBy(['nom' => $name]);
    }

    private function note(string $line): void
    {
        $this->plan[] = $line;
    }
}
