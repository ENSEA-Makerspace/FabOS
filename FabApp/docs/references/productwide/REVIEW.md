# Passe globale FabOS — constats et référentiel d’implémentation

## Invariants non négociables

1. **Une navigation admin unique.** Toute page, y compris RFID, logs, configuration et réseaux, utilise le même shell et le même footer.
2. **Un système de filtres unique.** Les états et lieux sont des filtres à un clic ; seule la recherche exige une saisie. Les filtres actifs sont lisibles et réinitialisables.
3. **Chaque objet ouvre sa fiche.** Les éléments des listes (prêt, matériau, événement, maintenance, utilisateur) gardent une cible de détail évidente.
4. **Action avant métrique.** Les tableaux de bord sont remplacés par une file de décisions/action ; les compteurs ne sont que du contexte.
5. **Thème global.** Logo, favicon, image, menu, widgets et homepage sont configurés depuis Configuration/Design ; kiosks et e-mails héritent du thème.
6. **Interface non gonflée.** Un écran exprime une tâche, une CTA primaire, les réglages avancés sont progressifs.

## Événements

- Le catalogue doit afficher lieu **et sous-lieu**, date, capacité, places restantes et statut d’inscription dès la carte.
- « À venir » doit avoir une définition stable avec un état vide explicatif et un bouton de réinitialisation ; jamais une disparition silencieuse.
- La fiche événement doit réunir inscription, billet, liste des inscrits (staff), tickets émis et actions de modification/annulation.
- Prévoir une vue staff opérationnelle pour check-in et attente, distincte de la page publique.

## Prêts, matériaux, maintenance

- Une ligne de prêt ouvre la fiche de l’objet au clic et comporte une seule action contextuelle (retour, relance, ouvrir).
- Matériaux doit rejoindre le domaine Équipement dans l’IA, sans confondre durable, consommable et pièce détachée. Afficher emplacement, compatibilité machine, seuil et fiche sécurité.
- Maintenance est une queue : intervention, échéance, indisponibilité, machine liée, checklist, pièces et trace de clôture.
- Catégories de machines doivent disposer de création, renommage, archivage/suppression protégée et du nombre de machines impactées.

## Configuration, design et contenu

- `/admin/settings` devient l’entrée par défaut de Configuration : blocs courts, état de configuration, pas de longue page disjointe.
- Fonctionnalités doit refléter workspaces et sous-ensembles. Email doit reprendre la même hiérarchie, avec aperçu et groupes de réglages courts.
- Design offre logo, favicon, image de thème, palette/préréglages, menu principal organisable, widgets de page d’accueil activables et choix d’une page custom comme accueil.
- Les Lab pages deviennent **Pages personnalisées** : contenu, visibilité, menu, page d’accueil et prévisualisation.
- Les actions rapides du dashboard peuvent être supprimées ; la prochaine action utile doit prendre leur place.

## Communauté, kiosks, réseau et droits

- Créations est une galerie de projets avec machines/matériaux employés et partage mesuré ; le leaderboard ne doit pas dominer l’expérience.
- Kiosk est une surface publique : hérite du thème et n’expose pas PII, UID de badge, secrets ou menu admin.
- Pour RFID et futur contrôle d’accès : une seule navigation, journaux exploitables, boîtiers par point d’accès (porte/portail) et **fail closed** sans secret valide.
- Droits doivent être expliqués par rôle, adhésion, formation, lieu/espace/machine et date d’expiration. Toute révocation/suspension est confirmée, motivée et journalisée.

## Priorité recommandée

**P0 — cohérence/sécurité** : shell admin+footer, traduction, filtres partagés, configuration par défaut, secret et fail-closed RFID, fiches de prêt cliquables.

**P1 — parcours quotidien** : événements (sous-lieux/billets/inscrits), matériel unifié, maintenance actionnable, catégories CRUD, thème appliqué aux kiosks/emails.

**P2 — éditorial/communauté** : pages personnalisées, accueil configurable, créations, améliorations de contenu et états vides.

## Prompt unique pour l’agent d’implémentation

> Fais une passe de consolidation FabOS à partir des référentiels visuels dans `docs/references/`. Travaille par composants et services partagés ; ne crée aucun menu, filtre, footer ou CSS local. Commence par P0 : shell admin unique, footer, traductions, filtre à un clic, fiche de prêt cliquable, Configuration comme entrée par défaut, et sécurité RFID fail-closed. Ensuite implémente les parcours P1 (événements, stock/équipement, maintenance, catégories, thème hérité). Vérifie pour chaque action le nombre de clics et que l’utilisateur peut modifier ce qui a été prévu sans interface gonflée. Ajoute/actualise les exemples `/admin/design`, tests fonctionnels, docs FabApp/docs ; déploie seulement les fichiers nécessaires sur Artemis, vérifie et commit chaque lot fini.
