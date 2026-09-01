<?php

declare(strict_types=1);

namespace App\UsageRights;

use App\Repository\MachineCategoryRepository;
use App\Reservation\ReservableType;

/**
 * Le COMPILATEUR de la saisie des packages (S153).
 *
 * Quatre lignes de restriction plus une d'extension, en entrée. En sortie : les
 * lignes de `USAGE_PACKAGE`, `USAGE_PACKAGE_FEATURE`, `USAGE_PACKAGE_GRANT`,
 * `USAGE_GRANT_WINDOW` et `USAGE_PACKAGE_ALLOWANCE` qui disent exactement ça.
 *
 * 🔴 **La normalisation est la raison d'être de ce fichier.** « Tout autoriser »
 * a coûté quatorze créations de grant sur l'installation mesurée, sur un package
 * dont `fullAccess` valait DÉJÀ 1 : deux vérités pour un seul fait, et le jour
 * où l'une des deux change, rien ne dit laquelle l'écran croit. Donc :
 * **aucune restriction sur les quatre axes, plus la cinquième ligne cochée, se
 * stocke en `fullAccess = 1` et RIEN d'autre** — pas une ligne de feature, pas
 * un grant, pas une fenêtre, pas un quota.
 *
 * ⚠️ **Et `fullAccess` est lu par les DEUX modèles depuis S153.**
 * `UsagePackageRepository::grantingPackages()` le lisait déjà (v1) ;
 * `UsageGrantRepository::paths()` l'ignorait (v2), si bien qu'un package
 * « accès complet » sans grants autorisait tout d'un côté et rien de l'autre.
 * La normalisation ci-dessus serait un piège sans cette correction : elle
 * produit précisément des packages `fullAccess` sans grants.
 *
 * ⚠️ **Hors de ce cas, les grants sont TOUJOURS écrits.** Sous la v2 un package
 * sans grant n'autorise rien, quelles que soient ses lignes de feature — la
 * liste v1 ne sert qu'aux chokepoints que l'opérateur n'a pas encore basculés.
 * Écrire les deux n'est pas une duplication : ce sont deux modèles, pas deux
 * copies, et le compilateur est le seul endroit qui les tienne d'accord.
 *
 * ⚠️ **L'action est toujours `Use`.** `Manage` n'est pas une case de cette
 * saisie : aucun des 21 grants mesurés ne la portait, et un package qui donne un
 * pouvoir d'administration se décrit avec des rôles, pas avec un forfait. Un
 * package qui en contient déjà un ne « rentre » plus dans la saisie et garde
 * l'éditeur détaillé — voir `decompile()`, qui rend `null` dans ce cas.
 */
final readonly class PackageSpecCompiler
{
    public function __construct(
        private UsagePackageRepository $packages,
        private UsageAllowanceRepository $allowances,
        private UsageCapabilityRegistry $capabilities,
        private MachineCategoryRepository $categories,
    ) {
    }

    /**
     * Écrire la spec sur un package existant. Remplace, n'ajoute pas.
     *
     * 🔴 **Remplacer est le seul comportement correct ici.** Le formulaire décrit
     * le package ENTIER : s'il ajoutait, décocher une case ne retirerait jamais
     * rien, et l'écran mentirait dans le sens le plus dangereux — celui qui
     * laisse un droit en place.
     */
    public function compile(int $packageId, string $name, string $description, bool $active, PackageSpec $spec): void
    {
        $keys = $this->capabilities->keys();
        $features = $spec->featuresAll ? $keys : array_values(array_intersect($keys, $spec->features));
        if ($features === []) {
            throw new \InvalidArgumentException('Un forfait doit donner accès à au moins une chose.');
        }

        // 🔴 La cinquième ligne ne peut pas coexister avec une restriction : elle
        // s'écrit dans un BIT, et un bit ne sait pas dire « tout sauf le jeudi ».
        // Le formulaire ne l'offre que quand les quatre axes sont ouverts ; ici
        // c'est la garde côté serveur, parce qu'un POST ne se laisse pas
        // convaincre par du CSS.
        if ($spec->hoursExempt && !$spec->isUnrestricted()) {
            throw new \InvalidArgumentException(
                "« Sans limite d'horaires » ne s'applique qu'à un package sans aucune autre restriction.",
            );
        }

        $unrestricted = $spec->isUnrestricted() && $spec->hoursExempt;

        $this->packages->save($packageId, $name, $description, $active, $unrestricted, $unrestricted ? [] : $features);

        if ($unrestricted) {
            $this->packages->replaceGrants($packageId, []);
            $this->allowances->replaceForPackage($packageId, []);

            return;
        }

        $this->packages->replaceGrants($packageId, $this->grantsFor($spec, $features));
        $this->allowances->replaceForPackage($packageId, $this->allowancesFor($spec, $features));
    }

    /**
     * Les grants d'une spec : un par (feature × lieu × catégorie), chacun portant
     * les mêmes fenêtres.
     *
     * ⚠️ **Le produit cartésien est ce que le modèle EXIGE**, pas une facilité :
     * les dimensions d'un grant se combinent en ET et les grants en OU, donc
     * « machines et espaces, sur deux lieux » est bien quatre lignes. Ce qui
     * change avec S153 est qui les écrit — un compilateur, en une soumission,
     * plutôt qu'un opérateur en quatorze.
     *
     * ⚠️ **La catégorie ne restreint QUE les machines.** Posée sur un grant
     * d'espaces ou d'événements, elle serait une restriction qui ne peut jamais
     * correspondre — le contrôleur la retirait déjà à la main pour cette raison,
     * ici c'est la boucle qui ne la propose pas.
     *
     * @param list<string> $features
     * @return list<array{featureKey:string,action:UsageGrantAction,venueId:?int,sectionKey:?string,reservableType:?string,reservableId:?int,categoryLabel:?string,categoryId:?int,windows:list<GrantWindow>}>
     */
    private function grantsFor(PackageSpec $spec, array $features): array
    {
        $windows = [];
        if (!$spec->daysAll) {
            foreach ($spec->days as $day) {
                if ($day >= 1 && $day <= 7) {
                    $windows[] = GrantWindow::fromClock($day, $spec->startTime, $spec->endTime);
                }
            }
            if ($windows === []) {
                throw new \InvalidArgumentException('Choisissez au moins un jour, ou laissez « tout le temps ».');
            }
        }

        $venueIds = $spec->venuesAll ? [null] : array_values(array_unique(array_map('intval', $spec->venues)));
        if ($venueIds === []) {
            throw new \InvalidArgumentException('Choisissez au moins un lieu, ou laissez « partout ».');
        }

        $categoryLabels = $spec->categoriesAll ? [null] : array_values(array_unique($spec->categories));
        if ($categoryLabels === []) {
            throw new \InvalidArgumentException('Choisissez au moins une catégorie, ou laissez « toutes ».');
        }

        $grants = [];
        foreach ($features as $feature) {
            $capability = $this->capabilities->get($feature);
            if ($capability === null) {
                continue;
            }
            // ⚠️ La catégorie est une phrase sur des MACHINES. Les autres
            // capacités reçoivent la ligne sans catégorie, sinon elles seraient
            // muettes au lieu d'être larges.
            $labels = $feature === 'machines' ? $categoryLabels : [null];
            foreach ($venueIds as $venueId) {
                foreach ($labels as $label) {
                    $grants[] = [
                        'featureKey' => $capability->featureKey,
                        'action' => UsageGrantAction::Use,
                        'venueId' => $venueId,
                        'sectionKey' => null,
                        // Une catégorie implique la famille : c'est une phrase sur
                        // les machines, donc elle porte son genre avec elle.
                        'reservableType' => $label === null ? null : ReservableType::Machine->value,
                        'reservableId' => null,
                        'categoryLabel' => $label,
                        // ⚠️ **L'identité en plus du libellé** (S147, J-21) : un
                        // renommage de catégorie décrochait le package en silence.
                        // Un libellé orphelin ne trouve pas de ligne, l'identifiant
                        // reste `null`, et le grant se comporte comme avant.
                        'categoryId' => $label === null ? null : $this->categories->findOneByLabel($label)?->getId(),
                        'windows' => $windows,
                    ];
                }
            }
        }

        return $grants;
    }

    /**
     * Le quota, en une ligne par feature réservable.
     *
     * ⚠️ **Heures à l'écran, minutes en base.** Un opérateur vend des heures et
     * une réservation se mesure en minutes ; convertir à la porte garde tous les
     * lecteurs de la table dans une seule unité.
     *
     * ⚠️ **Une ligne PAR feature, jamais une ligne globale.** Une allocation
     * sans `featureKey` compte toutes les réservations ensemble, si bien qu'une
     * heure d'espace mangerait le forfait machine — ce n'est pas ce que « au plus
     * 10 heures par semaine » veut dire dans la phrase du formulaire.
     *
     * @param list<string> $features
     * @return list<array{featureKey:?string,reservableType:?string,unit:string,amount:int,period:string}>
     */
    private function allowancesFor(PackageSpec $spec, array $features): array
    {
        if ($spec->quotaUnlimited) {
            return [];
        }
        $minutes = (int) round($spec->quotaHours * 60);
        if ($minutes <= 0) {
            throw new \InvalidArgumentException('Une allocation doit être strictement positive.');
        }
        if (!UsageAllowance::isValidPeriod($spec->quotaPeriod)) {
            throw new \InvalidArgumentException('Période inconnue.');
        }

        $rows = [];
        foreach ($features as $feature) {
            $capability = $this->capabilities->get($feature);
            if ($capability === null) {
                continue;
            }
            $rows[] = [
                'featureKey' => $capability->featureKey,
                'reservableType' => null,
                'unit' => UsageAllowance::UNIT_MINUTES,
                'amount' => $minutes,
                'period' => $spec->quotaPeriod,
            ];
        }

        return $rows;
    }

    /**
     * Relire un package existant COMME une spec, ou `null` s'il ne s'y réduit pas.
     *
     * 🔴 **Le refus est la partie importante.** Un package que la saisie ne sait
     * pas exprimer — un grant `Manage`, une section, une ressource précise, des
     * fenêtres qui diffèrent d'un grant à l'autre — ne doit pas être ouvert dans
     * un formulaire qui, à la première soumission, écraserait ce qu'il n'a pas su
     * afficher. La saisie se retire alors et l'éditeur détaillé reste. C'est la
     * même règle que « masquer n'est pas refuser », prise par l'autre bout :
     * **ne pas montrer ce qu'on ne saurait pas réécrire.**
     *
     * ⚠️ **Mais refuser trop est aussi un défaut, et la première version le
     * faisait.** Elle écartait tout package `fullAccess` portant des grants —
     * c'est-à-dire les DEUX seuls packages de la boîte, donc exactement ceux que
     * cette saisie vient ranger. La forme livrée ignore les grants inertes (voir
     * plus bas) et ne refuse que ce qu'elle ne saurait pas réécrire.
     */
    public function decompile(int $packageId, bool $fullAccess, array $features): ?PackageSpec
    {
        $keys = $this->capabilities->keys();
        $grants = $this->packages->grantsFor($packageId);

        $venues = [];
        $categories = [];
        $windowSignature = null;
        $windowSource = null;
        $grantFeatures = [];
        foreach ($grants as $grant) {
            // 🔴 **Les grants sur une feature qui n'est PAS une capacité sont
            // INERTES, et c'est mesuré.** Le package « Accès complet » de la
            // boîte en porte quatorze : `badges`, `loans`, `maintenance`,
            // `projects`, `staff`, `trainers`… Or `verdict()` traduit une
            // CAPACITÉ en clé de feature, et il n'en existe que quatre — les dix
            // autres lignes ne sont lues par rien. Elles viennent d'un backfill,
            // pas d'une décision.
            // ⚠️ Elles sont donc ignorées à la relecture ET remplacées à
            // l'écriture. C'est délibéré : les garder rendrait la saisie
            // inaccessible aux deux seuls packages qui existent, c'est-à-dire
            // exactement à ceux qu'elle vient ranger. Le jour où l'une de ces
            // clés gagne un chokepoint, elle gagne une capacité, et la saisie la
            // propose comme les autres.
            $capability = $this->capabilityForFeatureKey($grant['featureKey']);
            if ($capability === null) {
                continue;
            }
            if ($grant['action'] !== UsageGrantAction::Use->value
                || $grant['sectionKey'] !== null
                || $grant['reservableId'] !== null) {
                return null;
            }
            // Un genre de ressource sans catégorie n'est pas exprimable non plus :
            // la saisie ne propose « machines » que comme une FEATURE.
            if ($grant['reservableType'] !== null && $grant['categoryLabel'] === null) {
                return null;
            }
            $grantFeatures[$capability] = true;
            if ($grant['venueId'] !== null) {
                $venues[$grant['venueId']] = true;
            }
            if ($grant['categoryLabel'] !== null) {
                $categories[$grant['categoryLabel']] = true;
            }

            $signature = array_map(
                static fn (array $window): string => $window['dayOfWeek'] . '|' . $window['label'],
                $grant['windows'],
            );
            sort($signature);
            $signature = implode(',', $signature);
            // 🔴 Toutes les lignes doivent porter la MÊME semaine : la saisie n'a
            // qu'un seul axe « quand » pour tout le package, donc un package dont
            // deux grants ont des jours différents ne s'y réduit pas.
            if ($windowSignature !== null && $windowSignature !== $signature) {
                return null;
            }
            $windowSignature = $signature;
            $windowSource ??= $grant;
        }

        if ($grantFeatures === []) {
            // Aucun grant lisible : c'est un package neuf, un package v1 pur, ou
            // la forme normalisée `fullAccess` sans grants. Les features viennent
            // alors de la liste v1, et `fullAccess` veut dire « toutes ».
            return new PackageSpec(
                featuresAll: $fullAccess || $features === [] || count(array_intersect($keys, $features)) === count($keys),
                features: array_values(array_intersect($keys, $features)),
                hoursExempt: $fullAccess,
            );
        }

        $days = [];
        $start = null;
        $end = null;
        foreach (($windowSource['windows'] ?? []) as $window) {
            $days[] = (int) $window['dayOfWeek'];
            // `label` vaut « 14:00 – 18:00 », posé par `GrantWindow::label()`.
            // ⚠️ Deux plages HORAIRES différentes dans le même package ne sont
            // pas exprimables : la saisie a une heure de début et une de fin,
            // pour toute la semaine choisie.
            $parts = preg_split('/\s+–\s+/u', (string) $window['label']);
            if (!is_array($parts) || count($parts) !== 2) {
                return null;
            }
            if ($start !== null && ($start !== $parts[0] || $end !== $parts[1])) {
                return null;
            }
            [$start, $end] = $parts;
        }
        $days = array_values(array_unique($days));
        sort($days);

        $selected = array_values(array_intersect($keys, array_keys($grantFeatures)));
        $allowances = $this->allowances->forPackage($packageId);
        $quota = $this->quotaFrom($allowances, $selected);
        if ($quota === null && $allowances !== []) {
            return null;
        }

        $spec = new PackageSpec(
            featuresAll: count($selected) === count($keys),
            features: $selected,
            daysAll: $days === [],
            days: $days,
            startTime: $start ?? '09:00',
            endTime: $end ?? '18:00',
            venuesAll: $venues === [],
            venues: array_map('intval', array_keys($venues)),
            categoriesAll: $categories === [],
            categories: array_map('strval', array_keys($categories)),
            quotaUnlimited: $quota === null,
            quotaHours: $quota['hours'] ?? 10.0,
            quotaPeriod: $quota['period'] ?? 'week',
        );

        // ⚠️ **`fullAccess` ne se relit comme la cinquième ligne que si le reste
        // est ouvert.** Un package qui porte le bit ET des grants restreints est
        // incohérent — c'est la forme que cette phase range — et la saisie ne
        // peut pas afficher les deux à la fois. Elle affiche ce qu'elle SAIT
        // écrire, et enregistrer rend le package cohérent avec ce qui est à
        // l'écran. C'est la même règle que partout ici : ce qu'on voit est ce
        // qu'on enregistre.
        return $spec->isUnrestricted() && $fullAccess
            ? new PackageSpec(hoursExempt: true)
            : $spec;
    }

    /**
     * Le quota d'un package, s'il a la forme que la saisie sait écrire : une
     * ligne en MINUTES par feature accordée, toutes du même montant et de la même
     * période.
     *
     * @param list<UsageAllowance> $allowances
     * @param list<string> $features
     * @return array{hours: float, period: string}|null
     */
    private function quotaFrom(array $allowances, array $features): ?array
    {
        if ($allowances === [] || count($allowances) !== count($features)) {
            return null;
        }

        $amount = null;
        $period = null;
        foreach ($allowances as $allowance) {
            if ($allowance->unit !== UsageAllowance::UNIT_MINUTES || $allowance->reservableType !== null) {
                return null;
            }
            if ($amount !== null && ($amount !== $allowance->amount || $period !== $allowance->period)) {
                return null;
            }
            $amount = $allowance->amount;
            $period = $allowance->period;
        }

        return $amount === null || $period === null ? null : ['hours' => $amount / 60, 'period' => $period];
    }

    private function capabilityForFeatureKey(string $featureKey): ?string
    {
        foreach ($this->capabilities->all() as $key => $capability) {
            if ($capability->featureKey === $featureKey) {
                return $key;
            }
        }

        return null;
    }
}
