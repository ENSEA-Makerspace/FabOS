# Revue — parcours Utilisateur

## Proposition de parcours

**Visiteur** crée son compte → confirme son e-mail → complète les seules informations nécessaires → choisit une adhésion → est éventuellement validé par l’équipe → voit ses prochaines actions (paiement, formation, badge) → accède aux espaces/machines qui lui sont effectivement autorisés.

Le compte membre doit ensuite rester centré sur l’action suivante : poursuivre une formation, préparer une réservation, récupérer un badge ou renouveler une adhésion. Éviter un tableau de bord générique et des actions rapides redondantes.

## Principes de droits

- Séparer clairement rôle applicatif, adhésion, formation validée, accès aux lieux/espaces et autorisation machine.
- Afficher la **source** et l’**expiration** de chaque droit : l’utilisateur comme le staff comprend immédiatement pourquoi l’accès est permis ou refusé.
- Ne jamais afficher d’UID badge, de secret ou de journal d’accès dans l’espace membre.
- Les actions risquées (suspension, révocation, refus) demandent une justification et créent une trace ; elles ne vivent pas directement dans chaque ligne de tableau.
- Les filtres opérationnels sont à un clic (à valider, expire bientôt, suspendu). La recherche textuelle est la seule saisie.

## Prompt d’implémentation

> Agis comme product designer senior et ingénieur Symfony. Implémente un parcours Utilisateur FabOS cohérent de l’inscription à l’accès effectif, en utilisant le shell, les composants et les exemples de `/admin/design` existants ; aucun menu ou CSS local. Ajoute ou complète l’inscription, confirmation e-mail, onboarding minimal et adhésion selon les règles métier existantes. Crée un accueil membre focalisé sur la prochaine action utile, un profil avec sécurité/sessions/MFA, et une gestion des droits compréhensible : rôle, adhésion, formation, lieu/espace, machine, source et date d’expiration. Les actions de suspension/révocation/refus doivent être confirmées, justifiées et journalisées. Construis l’annuaire admin avec filtres à un clic et une vue de validation progressive. Ne pas exposer UID de badge, secrets, ni historiques sensibles aux membres. Tester les transitions de statut et autorisations ; mettre à jour FabApp/docs, déployer sélectivement Artemis, vérifier puis commit.
