<?php

namespace App\Controller;

use App\Form\MailOverrideType;
use App\Mail\MailLog;
use App\Mail\MailOverrides;
use App\Mail\MailSender;
use App\Mail\MailTemplateCatalog;
use App\Service\LocaleCatalog;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\Form\FormError;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;

/**
 * L'éditeur des textes d'e-mail (S161).
 *
 * 🔴 **Un écran par gabarit ET par langue.** Un texte réécrit par l'exploitant
 * est du CONTENU, et la règle de la maison est « on traduit l'UI, jamais le
 * contenu ». Un éditeur sans langue casserait les mails anglais d'un labo
 * bilingue qui n'aurait réécrit que le français — c'est écrit dans le plan de la
 * phase, avant la première ligne de code.
 *
 * ⚠️ **L'aperçu passe par `MailSender::render()`, la méthode qui ENVOIE.** La
 * mesure de sortie est « l'aperçu rend le vrai gabarit, pas une approximation » :
 * un second moteur de rendu pour la prévisualisation finirait par diverger de
 * celui qui envoie, et c'est exactement le défaut qu'un aperçu est censé
 * prévenir.
 *
 * 🅿️ **Le bouton « m'envoyer un test » n'est PAS ici.** `sendNow()` existe et
 * marcherait, mais poser un bouton qui envoie du vrai courrier depuis une
 * session automatisée n'est pas à moi de décider : c'est le genre d'action qu'on
 * ajoute quand quelqu'un peut la regarder partir. Consigné dans `ROADMAP.md`.
 */
#[Route('/admin/emails/gabarits')]
#[IsGranted('ROLE_ADMIN')]
final class MailTemplateAdminController extends AbstractController
{
    /**
     * ⚠️ **L'aperçu d'une partie de chrome se fait SUR un vrai mail.** Un
     * en-tête ou un pied n'existe pas seul : montrer la bande rose sans le
     * message autour ne dirait pas si le texte tient sur une ligne, ni si le
     * lien de désinscription est toujours là-dessous. `test` est le gabarit le
     * plus neutre — son contenu n'est le sujet de personne.
     */
    private const PART_PREVIEW = 'test';

    public function __construct(
        private readonly MailTemplateCatalog $catalog,
        private readonly MailOverrides $overrides,
        private readonly LocaleCatalog $locales,
        private readonly MailLog $log,
    ) {
    }

    #[Route('', name: 'app_admin_mail_templates', methods: ['GET'])]
    public function index(): Response
    {
        return $this->render('site/admin-mail-templates.html.twig', [
            'templates' => $this->catalog->names(),
            'parts' => $this->catalog->parts(),
            'locales' => $this->locales->codes(),
            'existing' => $this->overrides->existingKeys(),
            /*
             * 🔴 **Les surcharges qui se sont CASSÉES, dites là où on les
             * corrige (S162).** Le repli est silencieux par construction — c'est
             * ce qui garantit qu'un mot de passe oublié part quoi qu'il arrive —
             * mais silencieux jusqu'au bout signifierait qu'un exploitant voit
             * son texte enregistré ici et le texte livré dans sa boîte, sans
             * rien qui explique l'écart.
             * ⚠️ Dérivé du journal des envois, pas d'un drapeau stocké : un
             * drapeau serait un second état à remettre à zéro quand le texte est
             * réparé, et il se tromperait le jour où on oublie.
             */
            'fallbacks' => $this->log->fallbackKeys(),
        ]);
    }

    #[Route('/{name}/{locale}', name: 'app_admin_mail_template_edit', requirements: ['name' => '[a-z0-9_]+', 'locale' => '[a-z]{2}'], methods: ['GET', 'POST'])]
    public function edit(string $name, string $locale, Request $request, MailSender $sender): Response
    {
        // ⚠️ Deux listes blanches, pas une : un nom de gabarit inconnu
        // atteindrait `twig->load()` avec ce que l'URL veut, et une langue
        // inconnue écrirait une ligne que personne ne lira jamais.
        if (!$this->catalog->exists($name) || !in_array($locale, $this->locales->codes(), true)) {
            throw $this->createNotFoundException('Gabarit ou langue inconnus.');
        }

        $isPart = $this->catalog->isPart($name);

        $current = $this->overrides->find($name, $locale) ?? ['subject' => null, 'body' => null];
        $form = $this->createForm(MailOverrideType::class, [
            'subject' => $current['subject'] ?? '',
            'body' => $current['body'] ?? '',
        ]);

        // ⚠️ **Une partie de chrome n'a pas d'objet.** Le champ est RETIRÉ, pas
        // masqué : un champ présent mais caché finit par être soumis par un
        // navigateur ou un script, et on écrirait un objet que rien ne lit —
        // « surchargé » sans que rien ne change, l'état le plus difficile à
        // expliquer.
        if ($isPart) {
            $form->remove('subject');
        }

        $form->handleRequest($request);

        if ($form->isSubmitted()) {
            $data = $form->getData();

            /*
             * 🔴 **Le refus d'un champ inconnu, et il porte une PHRASE.** C'est
             * la mesure de sortie de la session. Un `{{ machine }}` dans le mail
             * d'un événement arriverait sinon écrit tel quel dans la boîte de
             * tout le monde — visible, donc pas dangereux, mais ridicule et
             * découvert par le destinataire plutôt que par l'auteur.
             * ⚠️ L'erreur est posée sur le CHAMP concerné, pas en haut de page :
             * un message global oblige à chercher lequel des deux il vise.
             */
            foreach ($isPart ? ['body'] : ['subject', 'body'] as $field) {
                $unknown = $this->catalog->unknownFieldsIn((string) ($data[$field] ?? ''), $name);
                if ($unknown !== []) {
                    $form->get($field)->addError(new FormError(sprintf(
                        'Ce gabarit ne connaît pas %s. Les champs disponibles sont listés à droite.',
                        implode(', ', array_map(static fn (string $f): string => '{{ ' . $f . ' }}', $unknown)),
                    )));
                }
            }

            /*
             * ⚠️ **Un objet sur deux lignes est un en-tête SMTP mal formé (S162).**
             * Le champ est un `<input>` et un navigateur en retire les retours,
             * mais un POST fabriqué à la main les laisse passer — et
             * `MailOverrides::save()` refuserait alors d'écrire sans pouvoir dire
             * quoi corriger. L'erreur est posée ici, sur le champ, avec la phrase.
             */
            if (!$isPart && preg_match('/[\r\n]/', (string) ($data['subject'] ?? '')) === 1) {
                $form->get('subject')->addError(new FormError('L’objet doit tenir sur une seule ligne : un e-mail ne peut pas porter un objet coupé en deux.'));
            }
        }

        if ($form->isSubmitted() && $form->isValid()) {
            $data = $form->getData();
            $ok = $this->overrides->save($name, $locale, (string) ($data['subject'] ?? ''), (string) ($data['body'] ?? ''));

            $this->addFlash($ok ? 'success' : 'error', $ok
                ? ['flash.gabarit_email_enregistre', ['%p1%' => $name, '%p2%' => $locale]]
                : 'flash.gabarit_email_non_enregistre');

            return $this->redirectToRoute('app_admin_mail_template_edit', ['name' => $name, 'locale' => $locale]);
        }

        /*
         * ⚠️ **L'aperçu est rendu avec un contexte D'EXEMPLE, et l'écran le dit.**
         * Un aperçu qui prétendrait montrer un vrai mail mentirait : le contexte
         * réel n'existe qu'au moment de l'envoi. Ce qu'il montre honnêtement,
         * c'est la MISE EN FORME et l'emplacement des champs.
         */
        $previewName = $isPart ? self::PART_PREVIEW : $name;
        $sample = $this->sampleContext($previewName);

        // ⚠️ Seulement pour une partie de chrome : c'est la seule façon de
        // MONTRER que le lien de désinscription survit à un pied réécrit. Un
        // gabarit ordinaire ne le reçoit que si sa catégorie est désactivable,
        // et l'inventer là fausserait l'aperçu.
        if ($isPart) {
            $sample['unsubscribe_url'] = 'https://exemple/desinscription';
        }

        $preview = null;
        try {
            [$subject, $html] = $sender->render($previewName, $sample, $locale);
            $preview = ['subject' => $subject, 'html' => $html];
        } catch (\Throwable $e) {
            // ⚠️ Un aperçu qui casse ne doit pas casser l'écran : l'auteur doit
            // pouvoir corriger son texte, et pour ça il lui faut le formulaire.
            $preview = ['subject' => null, 'html' => null, 'error' => $e->getMessage()];
        }

        return $this->render('site/admin-mail-template-edit.html.twig', [
            'name' => $name,
            'locale' => $locale,
            'form' => $form,
            'fields' => $this->catalog->fieldsOf($name),
            'preview' => $preview,
            'is_part' => $isPart,
            'overridden' => $this->overrides->find($name, $locale) !== null,
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    /**
     * ⚠️ **Des valeurs qui se VOIENT.** « ÉVÉNEMENT » en capitales dans l'aperçu
     * dit à l'auteur où atterrit le champ ; une valeur plausible (« Atelier du
     * mardi ») lui ferait croire qu'il regarde un vrai mail.
     *
     * @return array<string, mixed>
     */
    private function sampleContext(string $name): array
    {
        return [
            'event' => 'ÉVÉNEMENT', 'attendee' => 'PERSONNE', 'machine' => 'MACHINE',
            'formation' => 'FORMATION', 'subject' => 'OBJET', 'body' => 'CORPS',
            'author' => 'AUTEUR', 'item' => 'OBJET', 'place' => 'LIEU', 'task' => 'TÂCHE',
            'days' => 3, 'hours' => 24, 'validHours' => 2,
            'resetUrl' => 'https://exemple/reinitialiser',
            'date' => new \DateTimeImmutable('2026-01-02 09:00:00'),
            'start' => new \DateTimeImmutable('2026-01-02 10:00:00'),
            'end' => new \DateTimeImmutable('2026-01-02 11:00:00'),
            'dueDate' => new \DateTimeImmutable('2026-01-05 12:00:00'),
        ];
    }
}
