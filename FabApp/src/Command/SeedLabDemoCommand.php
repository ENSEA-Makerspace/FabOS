<?php

namespace App\Command;

use App\Entity\Badge;
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
        'Fraiseuse CNC', 'Oscilloscope Numérique', 'CNC 4x8', 'Uranus',
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

    public function __construct(private readonly EntityManagerInterface $em)
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
        $usinage = $this->em->getRepository(MachineCategory::class)->findOneBy(['label' => 'Usinage']);
        if ($usinage instanceof MachineCategory && $usinage->getArchivedAt() === null) {
            $this->note('catégorie archivée : Usinage (plus aucune machine : les deux CNC sont archivées)');
            $usinage->archive();
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

        foreach ($this->plan as $line) {
            $io->writeln('   ' . $line);
        }
        if ($dry) {
            $io->note(\count($this->plan) . ' changement(s) prévus — rien n’a été écrit (--dry-run).');

            return Command::SUCCESS;
        }
        $this->em->flush();
        $io->success(\count($this->plan) . ' changement(s) écrits.');

        return Command::SUCCESS;
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
