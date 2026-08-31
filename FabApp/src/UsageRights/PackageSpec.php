<?php

declare(strict_types=1);

namespace App\UsageRights;

/**
 * Ce qu'un package autorise, dit en QUATRE lignes de restriction plus une
 * d'extension (S153).
 *
 * 🔴 **Pourquoi cet objet existe.** Le modèle des droits a six dimensions —
 * feature, action, lieu, section, ressource, catégorie — plus des fenêtres et
 * des quotas dans deux autres tables. L'écran les faisait toutes remplir, une à
 * une, et la mesure du 2026-08-28 dit ce que ça a produit : **21 grants sur les
 * deux packages qui existent, et les 21 ont section, lieu, ressource et
 * catégorie VIDES.** Zéro fenêtre, zéro quota. Six dimensions offertes, aucune
 * employée, et « tout autoriser » payé en quatorze créations de grant.
 *
 * Cet objet est donc la SAISIE, pas le stockage : quatre axes qui se lisent
 * comme une phrase — *« ce package donne accès à tout, tout le temps, partout,
 * sans limite »* — que `PackageSpecCompiler` traduit dans le modèle. Le modèle
 * ne perd rien ; c'est la question posée qui change.
 *
 * ⚠️ **Chaque axe est un booléen « aucune restriction » PLUS sa liste.** Les
 * deux, et pas seulement la liste : « toutes les catégories » et « les cinq
 * catégories qui existent aujourd'hui » sont la même liste et deux intentions
 * différentes, et elles divergent le jour où l'opérateur en crée une sixième.
 * C'est la distinction que `NULL` porte dans la table des grants, et elle doit
 * survivre à l'aller-retour par le formulaire.
 */
final readonly class PackageSpec
{
    /**
     * @param list<string> $features clés de `UsageCapabilityRegistry`
     * @param list<int>    $days     1 (lundi) à 7 (dimanche)
     * @param list<int>    $venues   identifiants de `VENUE`
     * @param list<string> $categories libellés de catégorie de machines
     */
    public function __construct(
        public bool $featuresAll = true,
        public array $features = [],
        public bool $daysAll = true,
        public array $days = [],
        public string $startTime = '09:00',
        public string $endTime = '18:00',
        public bool $venuesAll = true,
        public array $venues = [],
        public bool $categoriesAll = true,
        public array $categories = [],
        public bool $quotaUnlimited = true,
        public float $quotaHours = 10.0,
        public string $quotaPeriod = 'week',
        /**
         * 🔴 **La cinquième ligne AJOUTE un pouvoir, les quatre autres en
         * retirent.** C'est pour ça qu'elle est décochée par défaut alors que
         * les quatre autres sont sur « aucune restriction », et pour ça qu'elle
         * n'a pas de liste : il n'y a rien à énumérer, seulement à lever.
         *
         * ⚠️ Elle ne lève que la **grille hebdomadaire**, jamais une fermeture
         * DATÉE — voir `ScheduleResolver::hasDatedRulesOn()`.
         */
        public bool $hoursExempt = false,
    ) {
    }

    /**
     * Les quatre axes sont-ils tous sur « aucune restriction » ?
     *
     * ⚠️ C'est la condition de la normalisation, et c'est aussi la condition
     * pour offrir la cinquième ligne : `fullAccess` est un seul bit, il ne peut
     * pas dire « tout sauf le jeudi ».
     */
    public function isUnrestricted(): bool
    {
        return $this->featuresAll && $this->daysAll && $this->venuesAll && $this->categoriesAll && $this->quotaUnlimited;
    }
}
