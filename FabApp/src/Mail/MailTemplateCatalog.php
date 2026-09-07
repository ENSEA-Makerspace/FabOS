<?php

namespace App\Mail;

/**
 * Quels gabarits d'e-mail existent, et **quels champs chacun met à disposition**
 * (S161).
 *
 * 🔴 **Sans cette liste, l'éditeur serait un piège.** Un exploitant qui écrit
 * `{{ machine }}` dans le mail d'un événement obtiendrait, au mieux, le champ
 * écrit tel quel dans le message reçu par tout le monde. La mesure de sortie de
 * S161 est explicite : « un champ inconnu est refusé avec une phrase ».
 *
 * ⚠️ **Les champs sont DÉRIVÉS de la source Twig, pas retapés dans une liste.**
 * Une liste tenue à la main diverge du jour où quelqu'un ajoute une variable à
 * un gabarit — et personne ne s'en aperçoit, parce que l'éditeur continue de
 * proposer l'ancienne. On lit le fichier, et ses inclusions.
 *
 * 🔴 **Ce que cette dérivation NE voit PAS, et qu'il faut dire** : un champ
 * qu'un gabarit n'utilise que dans une branche `{% if %}` est listé (bien), mais
 * un champ passé par un appelant PHP sans jamais apparaître dans le Twig ne
 * l'est pas. La liste est donc « ce que le gabarit sait afficher », ce qui est la
 * bonne définition pour un éditeur — pas « tout ce que le contexte contient ».
 */
final class MailTemplateCatalog
{
    /**
     * ⚠️ Toujours disponibles, quel que soit le gabarit : `MailSender` les ajoute
     * au contexte de chaque envoi. Les omettre ferait refuser un champ qui
     * marche.
     */
    private const ALWAYS = ['sender_name', 'unsubscribe_url'];

    /**
     * L'en-tête et le pied du layout, réécrivables SÉPARÉMENT (S162).
     *
     * 🔴 **Ce ne sont pas des gabarits, et ils ne sont pas non plus vingt fois
     * la même case.** L'en-tête et le pied sont les DEUX seules parties du
     * chrome communes aux 20 e-mails : les réécrire gabarit par gabarit
     * obligerait à saisir cent fois le même pied — et à le corriger cent fois.
     *
     * ⚠️ **Ils vivent dans la MÊME table, sous une clé réservée.** Une seconde
     * table pour deux lignes serait un second endroit où chercher « qui a
     * changé ce texte », et un second repli à écrire et à prouver.
     */
    public const PARTS = ['_header', '_footer'];

    /**
     * ⚠️ Ni les partiels ni le layout ne s'envoient : ils n'ont pas d'objet et
     * `_override` est l'enveloppe de la surcharge elle-même.
     */
    public function __construct(private readonly string $templateDir)
    {
    }

    /** @return list<string> */
    public function parts(): array
    {
        return self::PARTS;
    }

    public function isPart(string $name): bool
    {
        return in_array($name, self::PARTS, true);
    }

    /** @return list<string> les clés de gabarit, triées */
    public function names(): array
    {
        $names = [];
        foreach (glob($this->templateDir . '/*.html.twig') ?: [] as $path) {
            $base = basename($path, '.html.twig');
            if (str_starts_with($base, '_')) {
                continue;
            }
            $names[] = $base;
        }
        sort($names);

        return $names;
    }

    public function exists(string $name): bool
    {
        return $this->isPart($name) || in_array($name, $this->names(), true);
    }

    /**
     * Les champs que ce gabarit sait afficher.
     *
     * @return list<string>
     */
    public function fieldsOf(string $name): array
    {
        if (!$this->exists($name)) {
            return [];
        }

        /*
         * 🔴 **Le pied n'a PAS `unsubscribe_url` dans sa liste, et c'est
         * délibéré.** Le lien de désinscription est émis par le layout,
         * inconditionnellement, sous le texte du pied : l'exploitant réécrit la
         * phrase, il ne déplace ni ne retire la sortie de secours. Le proposer
         * comme champ laisserait croire l'inverse — et un pied qui l'aurait
         * « déplacé » puis perdu supprimerait une obligation légale par
         * inadvertance.
         * ⚠️ `sender_name` reste disponible : c'est le nom du labo, et c'est
         * précisément ce qu'un en-tête réécrit veut citer.
         */
        if ($this->isPart($name)) {
            return ['sender_name'];
        }

        $fields = self::ALWAYS;
        foreach ($this->sourcesOf($name) as $source) {
            /*
             * Deux formes, et ce sont les deux qu'utilisent ces gabarits :
             *   {{ event }}                                — impression directe
             *   |trans({'%event%': event})                 — dans une traduction
             * ⚠️ Le second est le cas MAJORITAIRE ici : le texte vit en clés de
             * traduction, donc un scan qui ne verrait que `{{ … }}` raterait
             * presque tout.
             */
            preg_match_all('/\{\{\s*([a-zA-Z_][a-zA-Z0-9_]*)\s*[|}]/', $source, $direct);
            preg_match_all('/%[a-zA-Z_][a-zA-Z0-9_]*%\'\s*:\s*([a-zA-Z_][a-zA-Z0-9_]*)/', $source, $viaTrans);
            $fields = array_merge($fields, $direct[1], $viaTrans[1]);
        }

        // ⚠️ Ce qui n'est pas un champ : les variables internes du gabarit et les
        // fonctions Twig. Une liste noire courte vaut mieux qu'un analyseur : la
        // conséquence d'un faux positif est un champ proposé qui rend vide, pas
        // une faille.
        $noise = ['loop', 'app', 'block', 'parent', 'override_subject', 'override_body', '_context', '_charset'];
        $fields = array_values(array_unique(array_diff($fields, $noise)));
        sort($fields);

        return $fields;
    }

    /**
     * La source du gabarit et celle des partiels qu'il inclut.
     *
     * ⚠️ Un niveau d'inclusion suffit : ces gabarits incluent `_event_details` et
     * consorts, qui n'incluent rien. Une récursion générale serait du code sans
     * cas d'usage, et sans garde contre un cycle.
     *
     * @return list<string>
     */
    private function sourcesOf(string $name): array
    {
        $main = @file_get_contents($this->templateDir . '/' . $name . '.html.twig');
        if ($main === false) {
            return [];
        }

        $sources = [$main];
        preg_match_all('/include\s+\'emails\/([a-z0-9_-]+)\.html\.twig\'/', $main, $includes);
        foreach ($includes[1] as $partial) {
            $body = @file_get_contents($this->templateDir . '/' . $partial . '.html.twig');
            if ($body !== false) {
                $sources[] = $body;
            }
        }

        return $sources;
    }

    /**
     * Les champs inconnus d'un texte, pour le refus de l'éditeur.
     *
     * @return list<string>
     */
    public function unknownFieldsIn(string $text, string $name): array
    {
        $known = $this->fieldsOf($name);
        preg_match_all('/\{\{\s*([a-zA-Z_][a-zA-Z0-9_]*)\s*\}\}/', $text, $used);

        return array_values(array_unique(array_diff($used[1], $known)));
    }
}
